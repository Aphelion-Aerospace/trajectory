%% post_launch_rail_velocity
% Author: Matthew Mader
% Email: maderm@purdue.edu
% Created: 3/17/2021
% Modified: 3/17/2021
%
% Description:
%   Post process and visualize the data generated from the
%   launch_rail_velocity Simulink model.
%% Clean workspace

clearvars,clc,close all

%% Run the model

tsim = 5; % [s] simulation length

% run the Simulink simulation
out = sim('launch_rail_velocity',tsim);

%% Format the data

t = out.tout; % [s] time vector

% unpack the state vector (x_bar)
x = out.x_bar.Data(:,1); % [m] horizontal position
y = out.x_bar.Data(:,2); % [m] vertical position
vx = out.x_bar.Data(:,3); % [m/s] inertial horizontal velocity
vy = out.x_bar.Data(:,4); % [m/s] inertial vertical velocity
theta = out.x_bar.Data(:,5); % [rad] attitude
q = out.x_bar.Data(:,6); % [rad/s] attitude rate
m = out.x_bar.Data(:,7); % [kg] mass

% unpack parts of the state vector derivative
ax = out.x_bar_dot.Data(:,3); % [m/s^2] inertial horizontal acceleration
ay = out.x_bar_dot.Data(:,4); % [m/s^2] inertial vertical acceleration
q_dot = out.x_bar_dot.Data(:,6); % [rad/s^2] attitude acceleration
m_dot = out.x_bar_dot.Data(:,7); % [kg/s] mass rate

%% Plot the data

% position plot
figure(1)
title("Rocket Position")
grid on
hold on
plot(t,x,'r-')
plot(t,y,'b-')
hold off
legend(["X" "Y"],'Location','best')
xlabel("Time [s]")
ylabel("Distance [m]")

% velocity plot
figure(2)
title("Rocket Velocity")
grid on
hold on
plot(t,vx,'r-')
plot(t,vy,'b-')
hold off
legend(["V_x" "V_y"],'Location','best')
xlabel("Time [s]")
ylabel("Velocity [m/s]")

% acceleration plot
figure(3)
title("Rocket Acceleration")
grid on
hold on
plot(t,ax,'r-')
plot(t,ay,'b-')
hold off
legend(["A_x" "A_y"],'Location','best')
xlabel("Time [s]")
ylabel("Acceleration [m/s^2]")

% attitude plot
figure(4)
title("Rocket Attitude")
grid on
hold on
plot(t,rad2deg(theta),'b-')
hold off
legend("\theta",'Location','best')
xlabel("Time [s]")
ylabel("Attitude [deg]")

% mass plot
figure(5)
title("Rocket Mass")
grid on
yyaxis left
plot(t,m,'b-')
ylabel("Mass [kg]")
yyaxis right
plot(t,m_dot,'r-')
ylabel("Mass Rate [kg/s]")
xlabel("Time [s]")

%% Find where velocity equals 100 ft/s

% find the vector norm along the rows
V = vecnorm([vx vy],1,2); % [m/s] velocity magnitude
V = convvel(V,'m/s','ft/s'); % [ft/s] unit conversion

% find the vector norm along the rows
L = vecnorm([x y],1,2); % [m] distance along launch rod
L = convlength(L,'m','ft'); % [ft] unit conversion

% velocity at 20 ft length
V_20ft = interp1(L,V,20,'linear')

% velocity at 30 ft length
V_30ft = interp1(L,V,30,'linear')

% length at 100 ft/s
L_stab = interp1(V,L,100,'linear')

