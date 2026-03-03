function xdot = dynamics(x, u, P)
% rw+thr only
% param
J = P.J; Jw = P.Jw; Cw = P.Cw; CT = P.CT; 
   
quat = x(1:4);
omega = x(5:7);
w_rate = x(8:11);
  
tau_w = u(1:4);
tau_T = u(5:6); 

%% Dynamics
tau_rw = -Cw*tau_w;
tau_ext = CT*tau_T; % dump
  
omega_dot = inv(J)*(-cross(omega, J*omega + Cw*Jw*w_rate ) + tau_rw + tau_ext );

% derivative
xdot(1:4,1) = 0.5*[0 -omega(1) -omega(2) -omega(3);
    omega(1) 0 omega(3) -omega(2);
    omega(2) -omega(3) 0 omega(1);
    omega(3) omega(2) -omega(1) 0] * quat;

xdot(5:7,1) = omega_dot;

xdot(8:11,1) = inv(Jw)*tau_w;
  

end
