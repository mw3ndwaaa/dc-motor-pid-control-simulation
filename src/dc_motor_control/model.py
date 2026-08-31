from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class DCMotorParameters:
    resistance: float = 1.0          # ohm
    inductance: float = 0.5          # H
    torque_constant: float = 0.01    # N m / A
    back_emf_constant: float = 0.01  # V s / rad
    inertia: float = 0.01            # kg m^2
    viscous_friction: float = 0.1    # N m s / rad

    def __post_init__(self):
        values = (
            self.resistance, self.inductance, self.torque_constant,
            self.back_emf_constant, self.inertia, self.viscous_friction,
        )
        if any(v <= 0 for v in values):
            raise ValueError("All DC motor parameters must be positive.")


class DCMotor:
    """Armature-controlled DC motor with states [current, angular_speed]."""

    def __init__(self, parameters: DCMotorParameters | None = None):
        self.p = parameters or DCMotorParameters()

    def derivatives(self, state, voltage: float, load_torque: float = 0.0):
        current, omega = np.asarray(state, dtype=float)
        p = self.p
        di_dt = (voltage - p.resistance * current - p.back_emf_constant * omega) / p.inductance
        domega_dt = (p.torque_constant * current - p.viscous_friction * omega - load_torque) / p.inertia
        return np.array([di_dt, domega_dt], dtype=float)

    def rk4_step(self, state, voltage: float, dt: float, load_torque: float = 0.0):
        if dt <= 0:
            raise ValueError("dt must be positive.")
        x = np.asarray(state, dtype=float)
        f = self.derivatives
        k1 = f(x, voltage, load_torque)
        k2 = f(x + 0.5 * dt * k1, voltage, load_torque)
        k3 = f(x + 0.5 * dt * k2, voltage, load_torque)
        k4 = f(x + dt * k3, voltage, load_torque)
        return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
