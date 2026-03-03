clear  
close all
clc;

simulation_time = 60;
dt = 0.01; % sec

% initial state
quat0 = [0.711; 0.319; -0.283; 0.559]; % [qs;qv]
target_quat = [-0.340; -0.610; 0.686; -0.203]; qd = target_quat;
omega0 = [0;-0.00111;0]; % [rad/s]
target_omega = [0;-0.00111;0]; % orbital angular rate

% PD
kp = 0.02;
kd = 0.08; 