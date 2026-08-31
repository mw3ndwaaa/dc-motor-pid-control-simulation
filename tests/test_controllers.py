from dc_motor_control import DCMotor, PIController, PIDController, simulate_closed_loop


def test_pi_reduces_steady_state_error():
    result = simulate_closed_loop(DCMotor(), PIController(18.0, 30.0), reference_speed=1.0, duration=4.0, dt=0.001)
    assert abs(1.0 - result.speed[-1]) < 0.03


def test_pid_output_is_limited():
    controller = PIDController(100.0, 100.0, 1.0, output_limits=(-12.0, 12.0))
    controller.reset()
    assert -12.0 <= controller.update(100.0, 0.01) <= 12.0
