function F = calculate_F_sensor_only(x, imu, dt)
% State: x = [p(3); v(3); eul(3)] with eul = [phi; theta; psi] (ZYX)
% IMU:   imu = [a_b(3); omega_b(3)] in body frame

phi = x(7); theta = x(8); psi = x(9);
a_b = imu(1:3);
omega_b = imu(4:6);

% Rotation and its angle partials (ZYX, body->nav)
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cps  = cos(psi); sps = sin(psi);

R = [ cth*cps,            cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
      cth*sps,            sps*sth*sphi + cps*cphi,  sps*sth*cphi - cps*sphi;...
      -sth,               cth*sphi,                 cth*cphi ];

% dR/dphi
dR_dphi = [ 0,                 cps*sth*cphi + sps*sphi,  -cps*sth*sphi + sps*cphi;...
            0,                 sps*sth*cphi - cps*sphi,  -sps*sth*sphi - cps*cphi;...
            0,                 cth*cphi,                 -cth*sphi ];

% dR/dtheta
dR_dth = [ -sth*cps,           cps*cth*sphi,             cps*cth*cphi;...
           -sth*sps,           sps*cth*sphi,             sps*cth*cphi;...
           -cth,               -sth*sphi,                -sth*cphi ];

% dR/dpsi
dR_dpsi = [ -cth*sps,          -sps*sth*sphi - cps*cphi, -sps*sth*cphi + cps*sphi;...
             cth*cps,           cps*sth*sphi - sps*cphi,  cps*sth*cphi + sps*sphi;...
             0,                 0,                        0 ];

% A = ∂(R a)/∂[phi,theta,psi]  (3x3)
A = [dR_dphi*a_b, dR_dth*a_b, dR_dpsi*a_b];

% Euler-rate matrix T(φ,θ) and its partials
costh = cth;  % guard if needed
if abs(costh) < 1e-6, costh = 1e-6*sign(costh + (costh==0)); end
tanth = sth/costh; sec2 = 1/(costh^2);

T = [ 1,      sphi*tanth,          cphi*tanth;...
      0,      cphi,                -sphi;...
      0,      sphi/costh,          cphi/costh ];

% dT/dphi
dT_dphi = [ 0,      cphi*tanth,        -sphi*tanth;...
            0,     -sphi,              -cphi;...
            0,      cphi/costh,        -sphi/costh ];

% dT/dtheta
dT_dth = [ 0,      sphi*sec2,          cphi*sec2;...
           0,      0,                  0;...
           0,      sphi*sth/(costh^2), cphi*sth/(costh^2) ];

% B = ∂(T ω)/∂[phi,theta,psi]  (3x3)  (note: no psi dependence)
B = [dT_dphi*omega_b, dT_dth*omega_b, [0;0;0]];

% Assemble F
F = eye(9);
F(1:3,4:6)   = dt*eye(3);      % p_k = p + v dt
F(4:6,7:9)   = dt*A;           % v_k sensitivity to angles via R(angles)*a
F(7:9,7:9)   = eye(3) + dt*B;  % angles kinematics via T(angles)*omega

% Small regularization for numerical safety
F = F + 1e-12*eye(9);
end