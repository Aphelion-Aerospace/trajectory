%% sim1dof
% Author: Matthew Mader
% Email: maderm@purdue.edu
% Created: 03/13/2021
% Modified: 03/13/2021
%
% Description:
%   Simple one degree of freedom rocket simulation for a demo on
%   rocket trajectory simulation. Rocket treated as a point mass
%   in vertical direction only with no rotation.

%% Clean Workspace

% clear variables and cmd window and close all open figures
clearvars,clc,close all

%% User Parameters

% rocket parameters (Estes 003226 HI-FLIER XL)
m_dry = 0.0632; % [kg] rocket dry mass
D_body = 0.042; % [m] rocket body diameter
Cd_body = 0.3; % [1] rocket body drag coefficient

% propulsion parameters (Estes E12-6)
load thrust_curves.mat
t_prop = E12.t; % [s] thrust curve time values
F_prop = E12.F; % [N] thrust curve force values
m_prop = 0.036; % [kg] propellent mass

% recovery parameters
D_par = 0.46; % [m] parachute diameter
t_par = 6; % [s] parachute deployment time
Cd_par = 0.75; % [1] parachute drag coefficient

% environment parameters
rho = 1.225; % [kg/m^3] air density
g = 9.8; % [m/s^2] gravitational acceleration

% simulation parameters
dt = 0.01; % [s] time step

%% Simple Simulation

% aerodynamic reference values
Sref_b = pi/4*D_body^2; % [m^2] rocket body reference area
Sref_p = pi/4*D_par^2; % [m^2] parachute reference area

% rocket mass
m_wet = m_dry + m_prop; % [kg] rocket wet mass

% initial conditions
t = 0; % [s] simulation time
h = 0; % [m] rocket altitude
v = 0; % [m/s] rocket velocity
a = 0; % [m/s^2] rocket acceleration
m = m_wet; % [kg] rocket mass


% main simulation loop
n = 2; % index counter
while h >= 0
    
    % increment time
    t(n) = t(n-1) + dt; % [s]
    
    % compute current rocket mass
    m(n) = interp1([0;t_prop(end)],[m_wet;m_dry],t(n),'linear',m_dry); % [kg]
    
    % compute forces acting on rocket
    Ft = interp1(t_prop,F_prop,t(n),'linear',0); % [N] thrust force
    Fg = m(n-1)*g; % [N] gravity force
%     Fd = 0.5*rho*v(n-1)^2*Cd_body*sign(v(n-1)); % [N] drag force
    Fd = 0;
    
%     Fnet = Ft - Fg - Fd; % [N] net force
    Fnet = Ft - Fd;
    
    disp(Fnet)
    
    % Newton's 2nd Law
    a(n) = Fnet/m(n) - 9.81;
    
    % estimate velocity
    v(n) = v(n-1) + a(n)*dt; % [m/s]
    
    % estimate altitude
    h(n) = h(n-1) + v(n)*dt; % [m]
    
    % increment counter
    n = n + 1;
    
end % while h > 0

%% Plot Results

figure(1)

% plot the altitude
subplot(2,2,1)
plot(t,h,'b-')
grid on
xlabel("Time [s]")
ylabel("Altitude [m]")
title("Rocket Altitude")

% plot the velocity
subplot(2,2,2)
plot(t,v,'b-')
grid on
xlabel("Time [s]")
ylabel("Velocity [m/s]")
title("Rocket Velocity")

% plot the acceleration
subplot(2,2,3)
plot(t,a,'b-')
grid on
xlabel("Time [s]")
ylabel("Acceleration [m/s^2]")
title("Rocket Acceleration")

% plot the mass
subplot(2,2,4)
plot(t,m,'b-')
grid on
xlabel("Time [s]")
ylabel("Mass [kg]")
title("Rocket Mass")

%% Better Simulation

[t,y] = rk4(@derivative,[0,10],[0;0],1000);

h = y(1,:);
v = y(2,:);

figure(2)
subplot(2,1,1)
plot(t,h)
subplot(2,1,2)
plot(t,v)


%% Functions
function dydt = derivative(t,y)

load thrust_curves.mat

t_prop = E12.t;
F_prop = E12.F;

m_wet = 0.0992;
m_dry = 0.0632;

Cd_body = 0.3;

g = 9.81;
rho = 1.225;

Ft = interp1(t_prop,F_prop,t,'linear',0);
m = interp1([0 t_prop(end)],[m_wet m_dry],t,'linear',m_dry);
% Fd = 0.5*rho*y(2)^2*Cd_body*sign(y(2));
Fd = 0;

Fnet = Ft - m*g - Fd;

a = Fnet/m;

dydt = [y(2); a];

end % function deriv
