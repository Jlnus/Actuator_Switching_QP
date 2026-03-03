function [next_time,next_state]= rk4(func_ref,X0,u,t,dt,param)
% Runge's fourth-order rule
% usage [ output state ] = rk4(@func_name,current_state,current_control,current_time,time_increment);
%
% Input variable
%   X0   = current state value
%   u    = current step control input
%   t    = current step time
%   dt   = time increment (if dt > 0; forward integratio, dt < 0; backward integration)
%   param = parameters to be passed to the equation of motion

% Input state is column vector
% output state is column vector, too

% state vector 
% x : Column vector;
% u : Column vector;

% initialize
next_state = [];

k1 = dt * feval(func_ref,X0,u,param);
k2 = dt * feval(func_ref,X0+0.5*k1,u,param);
k3 = dt * feval(func_ref,X0+0.5*k2,u,param);
k4 = dt * feval(func_ref,X0+k3,u,param);

next_state = ( X0 + ( k1 + 2*k2 + 2*k3 + k4 ) / 6 );
next_time = t+dt;
