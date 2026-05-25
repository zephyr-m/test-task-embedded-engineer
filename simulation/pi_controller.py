from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class PIConfig:
    kp: float
    ki: float
    integrator_min: float
    integrator_max: float


@dataclass
class PIState:
    integrator: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    output_raw: float = 0.0
    output: float = 0.0
    error: float = 0.0
    saturated: bool = False


class PIController:
    def __init__(self, config: PIConfig):
        self.config = config
        self.state = PIState()

    def preview(self, reference: float, measurement: float) -> float:
        self.state.error = reference - measurement
        self.state.p_term = self.config.kp * self.state.error
        self.state.i_term = self.state.integrator
        self.state.output_raw = self.state.p_term + self.state.integrator
        return self.state.output_raw

    def commit(self, dt: float, saturated_output: float) -> None:
        self.state.saturated = abs(saturated_output - self.state.output_raw) > 1e-9

        same_direction = (
            (self.state.error > 0.0 and self.state.output_raw > 0.0)
            or (self.state.error < 0.0 and self.state.output_raw < 0.0)
        )

        if not self.state.saturated or not same_direction:
            next_integrator = self.state.integrator + self.config.ki * self.state.error * dt
            self.state.integrator = clamp(
                next_integrator,
                self.config.integrator_min,
                self.config.integrator_max,
            )

        self.state.i_term = self.state.integrator
        self.state.output = saturated_output

    def reset(self) -> None:
        self.state = PIState()

