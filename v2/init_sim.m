clear;clc;
close all;

deg2rad = pi/180;

dt = 0.01;

%% Spacecraft & Actuators
P.J = [1257.52 -0.07285  -0.0345; % [kg m^2]
      -0.0728   11126.1   -24.1635;
      -0.0345  -24.1635   11354.6];

x0 = [1 2 3];