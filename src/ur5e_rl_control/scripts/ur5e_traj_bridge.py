#!/usr/bin/env python3
"""Load an Isaac Gym PyTorch policy and bridge /rl_state to /rl_action."""

import math
import os
import threading

import rospy
from std_msgs.msg import Float64MultiArray

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as functional
except ImportError as error:
    raise ImportError(
        "PyTorch is required by ur5e_traj_bridge.py. Install it in the Python "
        "environment used by ROS."
    ) from error


class CheckpointMLP(nn.Module):
    """Inference-only MLP assembled directly from checkpoint tensors."""

    def __init__(self, layers, activation):
        super().__init__()
        self.weights = nn.ParameterList(
            [nn.Parameter(weight.detach().clone(), requires_grad=False) for weight, _ in layers]
        )
        self.biases = nn.ParameterList(
            [
                nn.Parameter(bias.detach().clone(), requires_grad=False)
                for _, bias in layers
            ]
        )
        self.activation = activation

    def forward(self, observation):
        output = observation
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            output = functional.linear(output, weight, bias)
            if index + 1 < len(self.weights):
                output = self._activate(output)
        return output

    def _activate(self, value):
        if self.activation == "elu":
            return functional.elu(value)
        if self.activation == "relu":
            return functional.relu(value)
        if self.activation == "tanh":
            return torch.tanh(value)
        if self.activation in ("silu", "swish"):
            return functional.silu(value)
        raise RuntimeError("Unsupported policy activation: %s" % self.activation)


class IsaacGymPolicyBridge:
    def __init__(self):
        self._state_topic = rospy.get_param("~rl_state_topic", "/rl_state")
        self._action_topic = rospy.get_param("~rl_action_topic", "/rl_action")
        self._observation_dim = int(rospy.get_param("~policy_observation_dim", 18))
        self._action_dim = int(rospy.get_param("~policy_action_dim", 6))
        self._rate = float(rospy.get_param("~policy_rate", 20.0))
        self._state_timeout = float(rospy.get_param("~policy_state_timeout", 0.25))
        self._action_clip = float(rospy.get_param("~policy_action_clip", 1.0))
        self._observation_clip = float(
            rospy.get_param("~policy_observation_clip", 0.0)
        )
        self._activation = rospy.get_param("~policy_activation", "elu").lower()
        self._output_key = rospy.get_param("~policy_output_key", "")
        self._output_index = int(rospy.get_param("~policy_output_index", 0))
        requested_device = rospy.get_param("~policy_device", "cpu").lower()
        self._observation_indices = [
            int(value)
            for value in rospy.get_param(
                "~policy_observation_indices", list(range(self._observation_dim))
            )
        ]
        self._observation_offset = self._read_vector(
            "policy_observation_offset", self._observation_dim, 0.0
        )
        self._observation_scale = self._read_vector(
            "policy_observation_scale", self._observation_dim, 1.0
        )

        if self._observation_dim <= 0 or self._action_dim <= 0:
            raise rospy.ROSInitException("Policy dimensions must be positive")
        if len(self._observation_indices) != self._observation_dim:
            raise rospy.ROSInitException(
                "policy_observation_indices must match policy_observation_dim"
            )
        if min(self._observation_indices) < 0:
            raise rospy.ROSInitException("Observation indices cannot be negative")
        if self._rate <= 0.0 or self._state_timeout <= 0.0:
            raise rospy.ROSInitException("Policy rate and state timeout must be positive")
        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("CUDA requested but unavailable; falling back to CPU")
            requested_device = "cpu"
        self._device = torch.device(requested_device)

        configured_path = rospy.get_param("~policy_path", "")
        self._policy_path = os.path.abspath(
            os.path.expandvars(os.path.expanduser(configured_path))
        )
        if not configured_path or not os.path.isfile(self._policy_path):
            raise rospy.ROSInitException(
                "policy_path does not exist: %s" % configured_path
            )

        self._model, model_format = self._load_policy(self._policy_path)
        self._model.to(self._device)
        self._model.eval()

        self._lock = threading.Lock()
        self._latest_observation = None
        self._latest_state_time = None
        self._timeout_reported = False
        self._publisher = rospy.Publisher(
            self._action_topic, Float64MultiArray, queue_size=1
        )
        self._subscriber = rospy.Subscriber(
            self._state_topic,
            Float64MultiArray,
            self._state_callback,
            queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(1.0 / self._rate), self._infer)

        rospy.loginfo(
            "Loaded %s policy from %s on %s: obs=%d, action=%d, rate=%.1f Hz",
            model_format,
            self._policy_path,
            self._device,
            self._observation_dim,
            self._action_dim,
            self._rate,
        )
        rospy.logwarn(
            "Policy output is connected to %s. Keep hardware output disabled "
            "until observation ordering, normalization and actions are verified.",
            self._action_topic,
        )

    def _read_vector(self, name, size, default_value):
        values = rospy.get_param("~" + name, [default_value] * size)
        if len(values) != size:
            raise rospy.ROSInitException("%s must contain %d values" % (name, size))
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise rospy.ROSInitException("%s contains NaN or infinity" % name)
        return result

    def _load_policy(self, path):
        try:
            model = torch.jit.load(path, map_location=self._device)
            return model, "TorchScript"
        except (RuntimeError, ValueError):
            pass

        checkpoint = torch.load(path, map_location=self._device)
        if isinstance(checkpoint, nn.Module):
            return checkpoint, "nn.Module"
        if not isinstance(checkpoint, dict):
            raise rospy.ROSInitException(
                "Unsupported checkpoint type: %s" % type(checkpoint).__name__
            )

        module = self._find_module(checkpoint)
        if module is not None:
            return module, "checkpoint nn.Module"

        state_dict = self._find_state_dict(checkpoint)
        layers = self._extract_mlp_layers(state_dict)
        return CheckpointMLP(layers, self._activation), "state_dict MLP"

    @staticmethod
    def _find_module(checkpoint):
        for key in ("policy", "actor", "model"):
            value = checkpoint.get(key)
            if isinstance(value, nn.Module):
                return value
        return None

    def _find_state_dict(self, checkpoint):
        configured_key = rospy.get_param("~policy_state_dict_key", "")
        if configured_key:
            value = checkpoint.get(configured_key)
            if not isinstance(value, dict):
                raise rospy.ROSInitException(
                    "Checkpoint key '%s' is not a state_dict" % configured_key
                )
            return value

        if checkpoint and all(torch.is_tensor(value) for value in checkpoint.values()):
            return checkpoint
        for key in ("actor_state_dict", "model_state_dict", "state_dict", "model"):
            value = checkpoint.get(key)
            if isinstance(value, dict) and any(
                torch.is_tensor(item) for item in value.values()
            ):
                rospy.loginfo("Using checkpoint state_dict key: %s", key)
                return value
        raise rospy.ROSInitException(
            "No model/state_dict found in checkpoint. Available keys: %s"
            % sorted(checkpoint.keys())
        )

    def _extract_mlp_layers(self, state_dict):
        configured_keys = rospy.get_param("~policy_layer_keys", [])
        if configured_keys:
            base_keys = [str(key) for key in configured_keys]
        else:
            base_keys = self._auto_detect_layer_keys(state_dict)

        layers = []
        expected_input = self._observation_dim
        for base_key in base_keys:
            weight_key = base_key if base_key.endswith(".weight") else base_key + ".weight"
            base_key = weight_key[: -len(".weight")]
            weight = state_dict.get(weight_key)
            if not torch.is_tensor(weight) or weight.dim() != 2:
                raise rospy.ROSInitException("Missing 2-D tensor: %s" % weight_key)
            if int(weight.shape[1]) != expected_input:
                raise rospy.ROSInitException(
                    "%s expects %d inputs, previous layer provides %d"
                    % (weight_key, int(weight.shape[1]), expected_input)
                )
            bias = state_dict.get(base_key + ".bias")
            if bias is None:
                bias = torch.zeros(int(weight.shape[0]), dtype=weight.dtype)
            if not torch.is_tensor(bias) or bias.numel() != int(weight.shape[0]):
                raise rospy.ROSInitException("Invalid bias for layer %s" % base_key)
            layers.append((weight.float(), bias.float()))
            expected_input = int(weight.shape[0])

        if not layers or expected_input != self._action_dim:
            raise rospy.ROSInitException(
                "Actor layer chain must map observation dimension %d to action "
                "dimension %d. Set policy_layer_keys explicitly."
                % (self._observation_dim, self._action_dim)
            )
        rospy.loginfo("Actor layer keys: %s", base_keys)
        return layers

    def _auto_detect_layer_keys(self, state_dict):
        weighted = [
            key[: -len(".weight")]
            for key, value in state_dict.items()
            if key.endswith(".weight") and torch.is_tensor(value) and value.dim() == 2
        ]
        excluded = ("critic", "value", "sigma", "log_std", "running_mean")
        actor_keys = [
            key
            for key in weighted
            if any(token in key.lower() for token in ("actor", "policy", "mu"))
            and not any(token in key.lower() for token in excluded)
        ]
        candidates = actor_keys or [
            key for key in weighted if not any(token in key.lower() for token in excluded)
        ]

        chain = []
        current_dim = self._observation_dim
        for key in candidates:
            weight = state_dict[key + ".weight"]
            if int(weight.shape[1]) != current_dim:
                continue
            chain.append(key)
            current_dim = int(weight.shape[0])
            if current_dim == self._action_dim:
                return chain

        rospy.logerr("Available 2-D checkpoint weights: %s", weighted)
        raise rospy.ROSInitException(
            "Could not auto-detect actor layers. Configure policy_layer_keys."
        )

    def _state_callback(self, message):
        required_size = max(self._observation_indices) + 1
        if len(message.data) < required_size:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring RL state with %d values; observation mapping requires %d",
                len(message.data),
                required_size,
            )
            return
        raw = [float(message.data[index]) for index in self._observation_indices]
        if not all(math.isfinite(value) for value in raw):
            rospy.logwarn_throttle(2.0, "Ignoring RL state containing NaN/Inf")
            return
        observation = [
            (value - offset) * scale
            for value, offset, scale in zip(
                raw, self._observation_offset, self._observation_scale
            )
        ]
        if self._observation_clip > 0.0:
            limit = self._observation_clip
            observation = [max(-limit, min(limit, value)) for value in observation]
        with self._lock:
            self._latest_observation = observation
            self._latest_state_time = rospy.Time.now()
            self._timeout_reported = False

    def _infer(self, _event):
        now = rospy.Time.now()
        with self._lock:
            observation = (
                None
                if self._latest_observation is None
                else list(self._latest_observation)
            )
            state_time = self._latest_state_time
            timeout_reported = self._timeout_reported

        if observation is None:
            rospy.logwarn_throttle(2.0, "Waiting for the first valid RL state")
            return
        if (now - state_time).to_sec() > self._state_timeout:
            if not timeout_reported:
                rospy.logerr("RL state timed out; policy action publication stopped")
                with self._lock:
                    self._timeout_reported = True
            return

        tensor = torch.tensor(
            observation, dtype=torch.float32, device=self._device
        ).unsqueeze(0)
        try:
            with torch.no_grad():
                output = self._model(tensor)
                action = self._extract_action(output).reshape(-1)
        except (RuntimeError, TypeError, ValueError) as error:
            rospy.logerr_throttle(2.0, "Policy inference failed: %s", error)
            return
        if action.numel() != self._action_dim:
            rospy.logerr_throttle(
                2.0,
                "Policy returned %d actions, expected %d",
                action.numel(),
                self._action_dim,
            )
            return
        if not bool(torch.isfinite(action).all()):
            rospy.logerr_throttle(2.0, "Policy returned NaN/Inf actions")
            return
        if self._action_clip > 0.0:
            action = torch.clamp(action, -self._action_clip, self._action_clip)

        message = Float64MultiArray()
        message.data = action.detach().cpu().tolist()
        self._publisher.publish(message)

    def _extract_action(self, output):
        if torch.is_tensor(output):
            return output
        if isinstance(output, dict):
            if self._output_key:
                value = output.get(self._output_key)
                if torch.is_tensor(value):
                    return value
                raise TypeError(
                    "Configured policy_output_key '%s' is not a tensor"
                    % self._output_key
                )
            for key in ("actions", "action", "mu", "mus", "mean"):
                value = output.get(key)
                if torch.is_tensor(value):
                    return value
        if isinstance(output, (tuple, list)):
            if 0 <= self._output_index < len(output):
                value = output[self._output_index]
                if torch.is_tensor(value):
                    return value
            raise TypeError(
                "policy_output_index %d does not select a tensor"
                % self._output_index
            )
        raise TypeError("Cannot extract an action tensor from policy output")


if __name__ == "__main__":
    rospy.init_node("isaacgym_policy_bridge")
    IsaacGymPolicyBridge()
    rospy.spin()
