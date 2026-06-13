function y = sensor_imu(x, p, k, scfg) %#ok<INUSL>
%SENSOR_IMU  Noisy IMU + wheel-encoder measurement model (MPU6050-like).
%
%   y = SENSOR_IMU(x, p, k, scfg) returns a noisy measurement of the true
%   state x = [theta; theta_dot; phi_dot]:
%
%     theta_meas   = theta   + N(0, sigma_theta^2)      (accel/fused tilt)
%     thetad_meas  = theta_dot + gyro_bias + N(0, sigma_gyro^2)  (gyro: bias!)
%     phid_meas    = phi_dot + N(0, sigma_wheel^2)       (wheel speed estimate)
%
%   scfg fields: .sigma_theta, .sigma_gyro, .gyro_bias, .sigma_wheel (SI rad,
%   rad/s). The gyro bias is a constant offset -- the dominant real-world error
%   for an MPU6050 -- so the controller sees a steadily-wrong rate.
%
%   randn is used directly; seed with rng(...) upstream for reproducibility.

    theta = x(1);  thetad = x(2);  phid = x(3);

    y = [ theta  + scfg.sigma_theta * randn;
          thetad + scfg.gyro_bias + scfg.sigma_gyro * randn;
          phid   + scfg.sigma_wheel * randn ];
end
