function [t,y] = rk4(dydt,tspan,y0,n)
%%RK4 Runge-Kutta method numerical integration
%
% Description:
%   Better explicit numerical integration script implementing the
%   classic Runge-Kutta method.
%
% Input:
%   dydt - function handle f(t,y) = [y1',y2',...,ym']
%   tspan - time span [t0, tf]
%   y0 - state vector at t0 [y1(t0);y2(t0);...;ym(t0)]
%   n - number of integrations
%
% Output:
%   t - time vector [t0,t1,...,tf]
%   y - state vector vs time matrix [y0,y1,...,yn]
%
% Usage:
%   [t,y] = RK4(@(t,y)[y(2);-y(1)],[0,10],[1;0],100) - numerically
%   integrates the state space EOM for an undamped mass-spring
%   system.

%% Code

y = zeros(length(y0),n); % pre-allocate memory for state vectors
t = zeros(1,n); % pre-allocate memory for time values

t0 = tspan(1); % initial time
tf = tspan(2); % final time

dt = (tf - t0)/(n-1); % step size
t(1) = t0; % initialize time
y(:,1) = y0; % initial state vector

for i = 2:n
    t(i) = t(i-1) + dt; % increment time
    
    k1 = dydt(t(i-1),y(:,i-1)); % k1 = f(tn,yn)
    k2 = dydt(t(i-1)+dt/2,y(:,i-1)+dt*k1/2); % k2 = f(tn+h/2,yn+h*k1/2)
    k3 = dydt(t(i-1)+dt/2,y(:,i-1)+dt*k2/2); % k3 = f(tn+h/2,yn+h*k2/2)
    k4 = dydt(t(i-1)+dt,y(:,i-1)+dt*k3); % k4 = f(tn+h,yn+h*k3)
    
    % yn+1 = yn+ 1/6*h(k1 + 2*k2 + 2*k3 + k4)
    y(:,i) = y(:,i-1)+1/6*dt*(k1+2*k2+2*k3+k4);
end % for i

end % function rk4