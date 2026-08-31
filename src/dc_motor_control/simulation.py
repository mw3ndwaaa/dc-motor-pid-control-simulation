from dataclasses import dataclass
import numpy as np
from .model import DCMotor


@dataclass
class SimulationResult:
    time: np.ndarray
    current: np.ndarray
    speed: np.ndarray
    voltage: np.ndarray
    reference: np.ndarray


def _time_vector(duration: float, dt: float):
    if duration <= 0 or dt <= 0:
        raise ValueError("duration and dt must be positive.")
    return np.arange(0.0, duration + 0.5 * dt, dt)


def simulate_open_loop(motor: DCMotor, voltage: float = 12.0, duration: float = 3.0,
                       dt: float = 0.001, load_torque: float = 0.0):
    t = _time_vector(duration, dt)
    x = np.zeros(2)
    current = np.zeros_like(t)
    speed = np.zeros_like(t)
    volts = np.full_like(t, voltage, dtype=float)
    for k in range(1, len(t)):
        x = motor.rk4_step(x, voltage, dt, load_torque)
        current[k], speed[k] = x
    return SimulationResult(t, current, speed, volts, np.zeros_like(t))


def simulate_closed_loop(motor: DCMotor, controller, reference_speed: float = 1.0,
                         duration: float = 4.0, dt: float = 0.001,
                         load_torque: float = 0.0):
    t = _time_vector(duration, dt)
    x = np.zeros(2)
    current = np.zeros_like(t)
    speed = np.zeros_like(t)
    voltage = np.zeros_like(t)
    reference = np.full_like(t, reference_speed, dtype=float)
    controller.reset()
    for k in range(1, len(t)):
        error = reference_speed - x[1]
        u = controller.update(error, dt)
        x = motor.rk4_step(x, u, dt, load_torque)
        current[k], speed[k] = x
        voltage[k] = u
    return SimulationResult(t, current, speed, voltage, reference)
