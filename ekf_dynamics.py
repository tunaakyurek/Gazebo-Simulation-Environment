#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional
from ekf_parameters import EKFParameters

def rotation_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Compute ZYX (yaw-pitch-roll) body-to-world rotation matrix.
    
    Args:
        phi: roll angle (radians)
        theta: pitch angle (radians) 
        psi: yaw angle (radians)
    
    Returns:
        R: 3x3 rotation matrix from body to NED frame
    """
    # Safety check for numerical stability near pitch singularity
    if abs(np.cos(theta)) < 1e-6:
        print("Warning: Near-singularity detected in rotation matrix (cos(theta) near zero)")
        theta = np.sign(theta) * 1e-6  # small-angle replacement
    
    # Precompute sines and cosines
    cph = np.cos(phi)
    sph = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)
    cps = np.cos(psi)
    sps = np.sin(psi)
    
    # Construct R_z(psi) * R_y(theta) * R_x(phi)
    R = np.array([
        [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph],
        [sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph],
        [-sth,    cth*sph,               cth*cph]
    ])
    
    # Re-orthogonalize for numerical stability
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    
    return R

def drone_dynamics_imu(t: float, x: np.ndarray, imu: np.ndarray, 
                      params: EKFParameters) -> np.ndarray:
    """
    6-DOF strapdown mechanization used for EKF prediction, driven only by IMU.
    
    Args:
        t: time (unused in mechanization)
        x: state [pos(3); vel(3); att(3)] in NED, attitude [roll pitch yaw] (rad)
        imu: [accel_meas(3); gyro_meas(3)] in body frame
        params: parameters struct with fields including gravity g
    
    Returns:
        x_dot: time derivative of state for integration
    """
    # Unpack state
    pos = x[0:3]
    vel = x[3:6]
    att = x[6:9]  # [phi; theta; psi]
    
    # Unpack IMU measurements
    accel_meas = imu[0:3]  # body frame
    gyro_meas = imu[3:6]   # body frame
    
    g = params.g
    
    # Attitude kinematics using Euler rates (with singularity guard)
    phi, theta, psi = att[0], att[1], att[2]
    
    # Guard cos(theta) to avoid division by near-zero
    cos_theta = np.cos(theta)
    if abs(cos_theta) < 1e-6:
        cos_theta = 1e-6 * np.sign(cos_theta + (cos_theta == 0))
    
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    tan_theta = np.sin(theta) / cos_theta
    
    E = np.array([
        [1, sin_phi*tan_theta, cos_phi*tan_theta],
        [0, cos_phi, -sin_phi],
        [0, sin_phi/cos_theta, cos_phi/cos_theta]
    ])
    
    att_dot = E @ gyro_meas
    
    # Optional rate limiting for numerical stability
    max_rate = getattr(params, 'max_angular_rate', np.deg2rad(200))
    att_dot = np.clip(att_dot, -max_rate, max_rate)
    
    # Velocity dynamics: rotate body specific force to NED and add gravity
    R = rotation_matrix(phi, theta, psi)
    
    # In NED, gravity is a vector [0;0;+g]. Enforce positive-down convention.
    if np.isscalar(params.g):
        g_n = np.array([0.0, 0.0, params.g])
    else:
        gv = np.array(params.g).flatten()
        if len(gv) == 3:
            if gv[2] < 0:
                gv = -gv  # flip sign to ensure +g down
            g_n = gv
        else:
            g_n = np.array([0.0, 0.0, 9.81])
    
    vel_dot = R @ accel_meas + g_n
    
    # Position derivative
    pos_dot = vel
    
    # Assemble state derivative
    x_dot = np.zeros(9)
    x_dot[0:3] = pos_dot
    x_dot[3:6] = vel_dot
    x_dot[6:9] = att_dot
    
    return x_dot

def drone_dynamics(t: float, x: np.ndarray, u: np.ndarray, 
                  params: EKFParameters) -> np.ndarray:
    """
    6-DOF nonlinear drone model for truth simulation with simplified cross-coupling.
    
    Args:
        t: time (unused in dynamics itself)
        x: state [pos(3); vel(3); att(3)] in NED
        u: control [thrust; tau_phi; tau_theta; tau_psi]
        params: parameters struct containing mass, inertia, gravity, etc.
    
    Returns:
        x_dot: time derivative of the state
    """
    # Unpack state
    pos = x[0:3]
    vel = x[3:6]
    att = x[6:9]  # [phi; theta; psi]
    
    # Clamp actual roll and pitch to avoid singularities
    max_angle = np.deg2rad(10)  # 10 degrees for maximum stability
    att[0] = np.clip(att[0], -max_angle, max_angle)  # roll
    att[1] = np.clip(att[1], -max_angle, max_angle)  # pitch
    
    # Unpack control inputs
    T = u[0]  # Total thrust (N)
    tau = u[1:4]  # Torques (Nm)
    
    # Rotation matrix (body to NED)
    R = rotation_matrix(att[0], att[1], att[2])
    
    # Physical parameters
    m = params.mass
    I = params.I
    g = params.g
    
    # Forces
    f_gravity = m * g
    f_thrust = R @ np.array([0.0, 0.0, T])
    
    # Aerodynamic drag in body frame with strong damping
    vel_body = R.T @ vel  # Transform velocity to body frame
    drag_coeff = 0.5  # Doubled drag coefficient for extreme damping
    f_drag_body = -drag_coeff * np.abs(vel_body) * vel_body
    f_drag = R @ f_drag_body
    
    # Additional velocity limiting for safety (progressive damping)
    vel_mag = np.linalg.norm(vel)
    if vel_mag > 5.0:  # Much lower hard velocity limit
        vel_limit_factor = 5.0 / vel_mag
        vel = vel * vel_limit_factor
        # Add extremely strong damping force when approaching limit
        f_limit = -1.0 * m * (vel_mag - 4.0) * (vel / vel_mag)
        f_drag = f_drag + f_limit
    elif vel_mag > 3.0:  # Add progressive damping even at lower speeds
        f_limit = -0.5 * m * (vel_mag - 3.0) * (vel / vel_mag)
        f_drag = f_drag + f_limit
    
    # Centrifugal and Coriolis effects during turns (simplified)
    omega_body = np.array([0.0, 0.0, 0.0])  # Angular velocity in body frame (simplified)
    if np.linalg.norm(vel) > 0.1:  # Only apply when moving
        # Coriolis force: -2*m*omega_cross_vel
        omega_cross_vel = np.cross(omega_body, vel_body)
        f_coriolis = -2 * m * R @ omega_cross_vel
    else:
        f_coriolis = np.zeros(3)
    
    # Translational acceleration with cross-coupling
    acc = (f_thrust + f_gravity + f_drag + f_coriolis) / m
    
    # Angular rates mapping (Euler) with singularity handling
    phi, theta = att[0], att[1]
    epsilon = 1e-6
    
    # Improved transformation matrix with singularity handling
    if abs(np.cos(theta)) < epsilon:
        # Near singularity - use small angle approximation
        E = np.eye(3)
    else:
        E = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
    
    # Add gyroscopic effects (rotor angular momentum coupling)
    rotor_angular_momentum = np.array([0.0, 0.0, 0.1])  # Simplified rotor momentum
    gyro_torque = np.cross(omega_body, rotor_angular_momentum)
    
    # Total angular acceleration
    if np.linalg.cond(E) > 1e8:
        omega = np.zeros(3)
    else:
        # Include gyroscopic effects in angular dynamics
        tau_total = tau + gyro_torque
        omega = np.linalg.solve(E, tau_total / np.diag(I))
    
    # State derivatives
    x_dot = np.zeros(9)
    x_dot[0:3] = vel
    x_dot[3:6] = acc
    x_dot[6:9] = omega
    
    return x_dot

def calculate_F_sensor_only(x: np.ndarray, imu: np.ndarray, dt: float) -> np.ndarray:
    """
    Calculate state Jacobian for sensor-only EKF.
    
    Args:
        x: state [p(3); v(3); eul(3)] with eul = [phi; theta; psi] (ZYX)
        imu: [a_b(3); omega_b(3)] in body frame
        dt: time step
    
    Returns:
        F: 9x9 state transition Jacobian matrix
    """
    phi, theta, psi = x[6], x[7], x[8]
    a_b = imu[0:3]
    omega_b = imu[3:6]
    
    # Rotation and its angle partials (ZYX, body->nav)
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)
    cps = np.cos(psi)
    sps = np.sin(psi)
    
    R = np.array([
        [cth*cps, cps*sth*sphi - sps*cphi, cps*sth*cphi + sps*sphi],
        [cth*sps, sps*sth*sphi + cps*cphi, sps*sth*cphi - cps*sphi],
        [-sth, cth*sphi, cth*cphi]
    ])
    
    # dR/dphi
    dR_dphi = np.array([
        [0, cps*sth*cphi + sps*sphi, -cps*sth*sphi + sps*cphi],
        [0, sps*sth*cphi - cps*sphi, -sps*sth*sphi - cps*cphi],
        [0, cth*cphi, -cth*sphi]
    ])
    
    # dR/dtheta
    dR_dth = np.array([
        [-sth*cps, cps*cth*sphi, cps*cth*cphi],
        [-sth*sps, sps*cth*sphi, sps*cth*cphi],
        [-cth, -sth*sphi, -sth*cphi]
    ])
    
    # dR/dpsi
    dR_dpsi = np.array([
        [-cth*sps, -sps*sth*sphi - cps*cphi, -sps*sth*cphi + cps*sphi],
        [cth*cps, cps*sth*sphi - sps*cphi, cps*sth*cphi + sps*sphi],
        [0, 0, 0]
    ])
    
    # A = ∂(R a)/∂[phi,theta,psi] (3x3)
    A = np.column_stack([
        dR_dphi @ a_b,
        dR_dth @ a_b,
        dR_dpsi @ a_b
    ])
    
    # Euler-rate matrix T(φ,θ) and its partials
    costh = cth  # guard if needed
    if abs(costh) < 1e-6:
        costh = 1e-6 * np.sign(costh + (costh == 0))
    tanth = sth / costh
    sec2 = 1 / (costh**2)
    
    T = np.array([
        [1, sphi*tanth, cphi*tanth],
        [0, cphi, -sphi],
        [0, sphi/costh, cphi/costh]
    ])
    
    # dT/dphi
    dT_dphi = np.array([
        [0, cphi*tanth, -sphi*tanth],
        [0, -sphi, -cphi],
        [0, cphi/costh, -sphi/costh]
    ])
    
    # dT/dtheta
    dT_dth = np.array([
        [0, sphi*sec2, cphi*sec2],
        [0, 0, 0],
        [0, sphi*sth/(costh**2), cphi*sth/(costh**2)]
    ])
    
    # B = ∂(T ω)/∂[phi,theta,psi] (3x3) (note: no psi dependence)
    B = np.column_stack([
        dT_dphi @ omega_b,
        dT_dth @ omega_b,
        np.zeros(3)
    ])
    
    # Assemble F
    F = np.eye(9)
    F[0:3, 3:6] = dt * np.eye(3)  # p_k = p + v dt
    F[3:6, 6:9] = dt * A  # v_k sensitivity to angles via R(angles)*a
    F[6:9, 6:9] = np.eye(3) + dt * B  # angles kinematics via T(angles)*omega
    
    # Small regularization for numerical safety
    F = F + 1e-12 * np.eye(9)
    
    return F

def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))

def wrap_angles(angles: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi] range"""
    return np.arctan2(np.sin(angles), np.cos(angles))

if __name__ == '__main__':
    # Test functions
    from ekf_parameters import EKFParameters
    
    params = EKFParameters()
    
    # Test rotation matrix
    R = rotation_matrix(0.1, 0.2, 0.3)
    print(f"Rotation matrix:\n{R}")
    print(f"Determinant: {np.linalg.det(R)}")
    
    # Test dynamics
    x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])  # Initial state
    u = np.array([params.mass * 9.81, 0, 0, 0])  # Hover thrust
    imu = np.array([0, 0, -9.81, 0, 0, 0])  # IMU at rest
    
    x_dot = drone_dynamics(0, x, u, params)
    print(f"Dynamics x_dot: {x_dot}")
    
    x_dot_imu = drone_dynamics_imu(0, x, imu, params)
    print(f"IMU dynamics x_dot: {x_dot_imu}")
    
    # Test Jacobian
    F = calculate_F_sensor_only(x, imu, 0.01)
    print(f"Jacobian F shape: {F.shape}")
    print(f"Jacobian F condition number: {np.linalg.cond(F)}")
