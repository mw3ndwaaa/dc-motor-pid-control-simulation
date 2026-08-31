# Mathematical model

For an armature-controlled DC motor,

\[
V = L\frac{di}{dt} + Ri + K_e\omega
\]

and the mechanical dynamics are

\[
J\frac{d\omega}{dt} + b\omega = K_t i - \tau_L.
\]

The simulation uses the state vector

\[
x = \begin{bmatrix}i & \omega\end{bmatrix}^T
\]

and fourth-order Runge-Kutta integration.

The closed-loop controllers are

\[
u_P = K_p e,
\]

\[
u_{PI} = K_p e + K_i\int e\,dt,
\]

and

\[
u_{PID} = K_p e + K_i\int e\,dt + K_d\frac{de}{dt}.
\]

The implementation includes output saturation and simple anti-windup behavior.
