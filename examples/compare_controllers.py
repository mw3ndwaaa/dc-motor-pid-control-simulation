from pathlib import Path
import matplotlib.pyplot as plt

from dc_motor_control import (
    DCMotor, PController, PIController, PIDController,
    simulate_closed_loop, step_response_metrics,
)


def main():
    motor = DCMotor()
    reference = 1.0
    controllers = {
        "P": PController(kp=18.0),
        "PI": PIController(kp=18.0, ki=30.0),
        "PID": PIDController(kp=18.0, ki=30.0, kd=0.08),
    }

    results = {}
    print(f"Target speed: {reference:.2f} rad/s\n")
    print(f"{'Controller':<10}{'Rise (s)':>12}{'Overshoot (%)':>16}{'Settle (s)':>14}{'SSE':>12}")
    print("-" * 64)
    for name, controller in controllers.items():
        result = simulate_closed_loop(motor, controller, reference_speed=reference, duration=4.0, dt=0.001)
        results[name] = result
        m = step_response_metrics(result.time, result.speed, reference)
        print(f"{name:<10}{m['rise_time_s']:>12.4f}{m['overshoot_percent']:>16.2f}{m['settling_time_s']:>14.4f}{m['steady_state_error']:>12.5f}")

    fig, ax = plt.subplots(figsize=(9, 5.5))
    for name, result in results.items():
        ax.plot(result.time, result.speed, label=name)
    ax.axhline(reference, linestyle='--', linewidth=1.2, label='Reference')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular speed [rad/s]')
    ax.set_title('DC Motor Speed Control: P vs PI vs PID')
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()

    out = Path(__file__).resolve().parents[1] / 'assets' / 'controller_comparison.png'
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=180)
    print(f"\nSaved plot to {out}")


if __name__ == '__main__':
    main()
