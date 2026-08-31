import numpy as np
import pytest
from dc_motor_control import DCMotor, DCMotorParameters, simulate_open_loop


def test_zero_voltage_zero_state_has_zero_derivative():
    motor = DCMotor()
    assert np.allclose(motor.derivatives([0, 0], 0.0), [0, 0])


def test_positive_voltage_accelerates_motor():
    result = simulate_open_loop(DCMotor(), voltage=12.0, duration=0.5, dt=0.001)
    assert result.current[-1] > 0
    assert result.speed[-1] > 0


def test_invalid_parameters_rejected():
    with pytest.raises(ValueError):
        DCMotorParameters(resistance=-1.0)
