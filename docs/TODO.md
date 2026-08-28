# TODO

Next task: add pitch-rate and yaw-rate feedback in addition to pitch and yaw.

`controller.control_step` currently builds the measurement as

`y = [theta, psi]`

(fused pitch and relative yaw, in radians). The H-infinity controller never
sees `theta_dot` or `psi_dot`. Raw gyro rates from `fuse_sample` (`sample["gyro"]`)
and numerical derivatives of fused angles are unused.

That is a likely reason the loop can feel laggy even when the IMU/gyro itself
is fast: the control law is angle-only.

Plan:

1. Estimate pitch rate and yaw rate (body gyro in the tilt/yaw axes, or
   `d/dt` of fused pitch/yaw with the same `dt` as the 100 Hz loop).
2. Extend `y` (and the H-infinity matrices, if the design assumed extra
   measurements) so `u` depends on rates as well as angles.
3. Keep the existing 10 ms `Ts`, print helper off by default, and unit tests.
