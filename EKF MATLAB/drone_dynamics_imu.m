function [x_dot] = drone_dynamics_imu(t, x, imu, params)
%% PURPOSE
% 6-DOF strapdown mechanization used for EKF prediction, driven only by IMU.
%
% INPUTS
% - t      : time (unused in mechanization)
% - x      : state [pos(3); vel(3); att(3)] in NED, attitude [roll pitch yaw] (rad)
% - imu    : [accel_meas(3); gyro_meas(3)] in body frame
% - params : struct with fields including gravity g and optional limits
%
% OUTPUTS
% - x_dot  : time derivative of state for integration
%
% MAJOR STEPS
% 1) Compute attitude rates from gyro with Euler kinematics and singularity guard
% 2) Rotate measured specific force to NED and add gravity to get vel_dot
% 3) pos_dot = vel, then assemble x_dot
%
% Frames: Body (b) and NED (n). Rotation R = R_bn maps body→NED.

%% 1) Unpack state
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

%% 2) Unpack IMU measurements
accel_meas = imu(1:3); % body frame
gyro_meas = imu(4:6);  % body frame

g = params.g;

%% 3) Attitude kinematics using Euler rates (with singularity guard)
phi = att(1); theta = att(2); psi = att(3);

% Guard cos(theta) to avoid division by near-zero
cos_theta = cos(theta);
if abs(cos_theta) < 1e-6
    cos_theta = 1e-6 * sign(cos_theta + (cos_theta==0));
end

sin_phi = sin(phi); cos_phi = cos(phi);
tan_theta = sin(theta)/cos_theta;

E = [1, sin_phi*tan_theta,  cos_phi*tan_theta;
     0, cos_phi,            -sin_phi;
     0, sin_phi/cos_theta,   cos_phi/cos_theta];

att_dot = E * gyro_meas;

% Optional rate limiting for numerical stability
if isfield(params, 'max_angular_rate')
    max_rate = params.max_angular_rate;
else
    max_rate = deg2rad(200);
end
att_dot = max(min(att_dot, max_rate), -max_rate);

%% 4) Velocity dynamics: rotate body specific force to NED and add gravity
R = rotation_matrix(phi, theta, psi);
% In NED, gravity is a vector [0;0;+g]. Enforce positive-down convention.
if isscalar(params.g)
    g_n = [0; 0; params.g];
else
    gv = params.g(:);
    if numel(gv) == 3
        if gv(3) < 0
            gv = -gv; % flip sign to ensure +g down
        end
        g_n = gv;
    else
        g_n = [0; 0; 9.81];
    end
end
vel_dot = R * accel_meas + g_n;

%% 5) Position derivative
pos_dot = vel;

%% 6) Assemble state derivative
x_dot = zeros(9,1);
x_dot(1:3) = pos_dot;
x_dot(4:6) = vel_dot;
x_dot(7:9) = att_dot;
end 