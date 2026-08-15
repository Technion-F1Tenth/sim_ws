"""
Self-contained PyTorch definition of the roboracer policy architecture.

Extracted verbatim (class bodies unchanged) from PufferLib's
`pufferlib/models.py` (branch 4.0, vendored into the puffertank container at
/puffertank/pufferlib) so `policy.pt` can be loaded WITHOUT installing
PufferLib. This is the "--slowly" backend's plain-nn.Module implementation —
the only one whose checkpoints are ordinary torch state dicts (the default
fast backend fuses everything into hardcoded CUDA kernels in src/models.cu
and its `.bin` checkpoints are a raw parameter dump, not torch-loadable).

See MODEL_INFO.md in this directory for observation/action layout,
checkpoint provenance, and usage notes.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

OBS_SIZE = 257       # N_BEAMS(64) * LIDAR_STACK_N(4) + 1 (velocity)
HIDDEN_SIZE = 128
NUM_GRU_LAYERS = 4
ACTION_DIM = 2        # [steering_velocity_cmd (rad/s), acceleration_cmd (m/s^2)]


class DefaultEncoder(nn.Module):
    """Flattens the observation and runs it through a single Linear layer."""

    def __init__(self, obs_size, hidden_size=128):
        super().__init__()
        self.encoder = nn.Linear(obs_size, hidden_size)

    def forward(self, observations):
        return self.encoder(observations.view(observations.shape[0], -1).float())


class MinGRU(nn.Module):
    """
    Minimal GRU recurrent core (https://arxiv.org/abs/2410.01201v1).

    Log-space parallel scan for training (`forward_train`); a simple
    per-step recurrence for rollout/inference (`forward_eval`), which is the
    one deployment code should call.
    """

    def __init__(self, hidden_size, num_layers=1, **kwargs):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.layers = nn.ModuleList([
            nn.Linear(hidden_size, 3 * hidden_size, bias=False) for _ in range(num_layers)
        ])

    def _g(self, x):
        return torch.where(x >= 0, x + 0.5, x.sigmoid())

    def _log_g(self, x):
        return torch.where(x >= 0, (F.relu(x) + 0.5).log(), -F.softplus(-x))

    def _highway(self, x, out, proj):
        g = proj.sigmoid()
        return g * out + (1.0 - g) * x

    def _heinsen_scan(self, log_coeffs, log_values):
        a_star = log_coeffs.cumsum(dim=1)
        return (a_star + (log_values - a_star).logcumsumexp(dim=1)).exp()

    def initial_state(self, batch_size, device):
        return (torch.zeros(self.num_layers, batch_size, self.hidden_size, device=device),)

    def forward_eval(self, h, state):
        """Single timestep. h: [B, hidden]. state: (h_prev,), h_prev [layers, B, hidden]."""
        state = state[0]
        assert state.shape[1] == h.shape[0]
        h = h.unsqueeze(1)
        state_out = []
        for i in range(self.num_layers):
            hidden, gate, proj = self.layers[i](h).chunk(3, dim=-1)
            out = torch.lerp(state[i:i + 1].transpose(0, 1), self._g(hidden), gate.sigmoid())
            h = self._highway(h, out, proj)
            state_out.append(out[:, -1:])
        return h.squeeze(1), (torch.stack(state_out, 0).squeeze(2),)

    def forward_train(self, h):
        T = h.shape[1]
        for i in range(self.num_layers):
            hidden, gate, proj = self.layers[i](h).chunk(3, dim=-1)
            log_coeffs = -F.softplus(gate)
            log_values = -F.softplus(-gate) + self._log_g(hidden)
            out = self._heinsen_scan(log_coeffs, log_values)[:, -T:]
            h = self._highway(h, out, proj)
        return h


class DefaultDecoder(nn.Module):
    """Continuous Normal(mean, std) action head + a scalar value head."""

    def __init__(self, nvec, hidden_size=128):
        super().__init__()
        self.nvec = tuple(nvec)
        self.is_continuous = sum(nvec) == len(nvec)

        if self.is_continuous:
            num_atns = len(nvec)
            self.decoder_mean = nn.Linear(hidden_size, num_atns)
            self.decoder_logstd = nn.Parameter(torch.zeros(1, num_atns))
        else:
            self.decoder = nn.Linear(hidden_size, int(sum(nvec)))

        self.value_function = nn.Linear(hidden_size, 1)

    def forward(self, hidden):
        if self.is_continuous:
            mean = self.decoder_mean(hidden)
            logstd = self.decoder_logstd.expand_as(mean)
            logits = torch.distributions.Normal(mean, torch.exp(logstd))
        else:
            logits = self.decoder(hidden)
            if len(self.nvec) > 1:
                logits = logits.split(self.nvec, dim=1)

        values = self.value_function(hidden)
        return logits, values


class Policy(nn.Module):
    """encoder -> recurrent core -> decoder, matching the checkpoint's key prefixes."""

    def __init__(self, encoder, decoder, network):
        super().__init__()
        self.encoder = encoder
        self.decoder = decoder
        self.network = network

    def initial_state(self, batch_size, device):
        return self.network.initial_state(batch_size, device)

    def forward_eval(self, x, state):
        """x: [B, OBS_SIZE] observation. Returns (Normal dist, value, new_state)."""
        h = self.encoder(x)
        h, state = self.network.forward_eval(h, state)
        logits, values = self.decoder(h)
        return logits, values, state

    def forward(self, x):
        """Training-time forward over a [B, T, OBS_SIZE] sequence (parallel scan, no state)."""
        B, TT = x.shape[:2]
        h = self.encoder(x.reshape(B * TT, *x.shape[2:]))
        h = self.network.forward_train(h.reshape(B, TT, -1))
        logits, values = self.decoder(h.reshape(B * TT, -1))
        return logits, values.reshape(B, TT)


def build_policy(obs_size=OBS_SIZE, hidden_size=HIDDEN_SIZE,
                 num_layers=NUM_GRU_LAYERS, action_dim=ACTION_DIM):
    """Construct an untrained Policy with the exact architecture policy.pt was saved from."""
    encoder = DefaultEncoder(obs_size, hidden_size)
    network = MinGRU(hidden_size, num_layers=num_layers)
    decoder = DefaultDecoder(nvec=[1] * action_dim, hidden_size=hidden_size)
    return Policy(encoder, decoder, network)


def load_policy(checkpoint_path="policy.pt", device="cpu"):
    """Build the architecture and load trained weights. Returns a Policy in eval() mode."""
    policy = build_policy()
    state_dict = torch.load(checkpoint_path, map_location=device, weights_only=True)
    policy.load_state_dict(state_dict)
    policy.to(device)
    policy.eval()
    return policy


if __name__ == "__main__":
    # Smoke test: load the checkpoint and run one dummy timestep.
    policy = load_policy()
    state = policy.initial_state(batch_size=1, device="cpu")
    dummy_obs = torch.zeros(1, OBS_SIZE)
    dummy_obs[:, -1] = 0.3  # v/v_max
    with torch.no_grad():
        dist, value, state = policy.forward_eval(dummy_obs, state)
    print("action mean:", dist.mean.numpy())
    print("action std: ", dist.stddev.numpy())
    print("value:      ", value.item())
