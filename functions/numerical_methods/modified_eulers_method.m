function [t,y] = modified_eulers_method(dydt,tspan,y0,n)
%%MODIFIED_EULERS_METHOD Euler's method numerical integration
%
% Description:
%   Slightly better explicity numerical integration technique
%   introducing predictor-corrector methods to Euler's Method.
%
% Input:
%   dydt: function handle f(t,y) = [y1',y2',...,ym']
%   tspan: time span [t0, tf]
%   y0: state vector at t0 [y1(t0);y2(t0);...;ym(t0)]
%   n: number of integrations
%
% Output:
%   t: time vector [t0,t1,...,tf]
%   y - state vector vs time matrix [y0,y1,...,yn]
%
% Usage:
%   [t,y] = MODIFIED_EULERS_METHOD(@(t,y)[y(2);-y(1)],[0,10],[1;0],100)
%   numerically integrates the state space EOM for an undamped mass-spring
%   system.

%% Code

y = zeros(length(y0),n); % pre-allocate memory for state vectors
t = zeros(1,n); % pre-allocate memory for time values

t0 = tspan(1); % initial time
tf = tspan(2); % final time
dt = (tf-t0)/(n-1); % time step

t(1) = t0; % initialize time
y(:,1) = y0; % initial state vector
for i = 2:n
    t(i) = t(i-1) + dt; % increment time
    y(:,i) = y(:,i-1) + dt/2*(dydt(t(i-1),y(:,i-1)) + ...
        dydt(t(i-1) + dt,y(:,i-1) + dt*dydt(t(i-1),y(:,i-1))));
end % for i
end % function modified_eulers_method