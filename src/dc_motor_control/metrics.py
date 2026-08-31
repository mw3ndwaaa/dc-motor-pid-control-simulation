import numpy as np


def step_response_metrics(time, response, reference: float, settling_band: float = 0.02):
    t = np.asarray(time, dtype=float)
    y = np.asarray(response, dtype=float)
    if t.size != y.size or t.size < 2:
        raise ValueError("time and response must have the same length >= 2.")
    if reference == 0:
        raise ValueError("reference must be non-zero.")

    target10 = 0.1 * reference
    target90 = 0.9 * reference
    if reference > 0:
        i10 = np.flatnonzero(y >= target10)
        i90 = np.flatnonzero(y >= target90)
    else:
        i10 = np.flatnonzero(y <= target10)
        i90 = np.flatnonzero(y <= target90)
    rise_time = float('nan')
    if i10.size and i90.size:
        rise_time = float(t[i90[0]] - t[i10[0]])

    peak = float(np.max(y)) if reference > 0 else float(np.min(y))
    overshoot = max(0.0, (peak - reference) / abs(reference) * 100.0) if reference > 0 else max(0.0, (reference - peak) / abs(reference) * 100.0)

    band = settling_band * abs(reference)
    outside = np.flatnonzero(np.abs(y - reference) > band)
    settling_time = 0.0 if outside.size == 0 else (float(t[outside[-1] + 1]) if outside[-1] + 1 < len(t) else float('nan'))
    steady_state_error = float(reference - y[-1])
    return {
        "rise_time_s": rise_time,
        "overshoot_percent": overshoot,
        "settling_time_s": settling_time,
        "steady_state_error": steady_state_error,
    }
