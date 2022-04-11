% %% Turning system parameters
% 
% K_phi = 1;
% sigma_phi = 10;
% 
% %% Forward system parameters
% 
% K_rho = 1;
% sigma_rho = 10;
% 
% %% Forward Speed Setpoint
% 
% rhodot_d = 0.1; % [ft/s]
% 
% %% Initial Conditions
% 
% x_0 = 0; y_0 = 0; phi_0 = 0;
% 
% %% Line Parameters
% x_s = [0 3 6 10];
% y_s = [0 0 3 3];
% 
% %%
% 
% sim('Demo2ControlSimulation')
% 
% animate
% 
% %%%   

%%Step Experiment to find inner Loop controller
% Group Team 9 
%
% This script shows results from both the experimental and simulated open
% loop step response experiments for Demo 1
%
% Required File: RunDemo2InnerControl.slx
%
%% Open loop step response
% Import the data from the step experiment 
% [Acquired using Arduino code and excel]
data = importdata('StepData.csv');
timeD = ((data(96:end, 1)) - 1024)/100;

% Right Wheel Data
thetaDot1 = data(96:end, 2);

% Left Wheel Data
thetaDot2 = data(96:end, 3);

%%Plot Velocity vs time for both wheels
figure(1)
plot(timeD, thetaDot1, 'r')
hold on

figure(2)
plot(timeD, thetaDot2, 'r')
hold on

% Define paramters for simulated transfer function for Right Wheel
k_1 = (9.76) / 4;
sigma_1 = 15.1;
sys_1 = tf(k_1*sigma_1, [1 sigma_1]);
step(sys_1 * 4, 'b');
title("Open Step Response");
xlabel('Time');
ylabel('Velocity (rad/s)');

% Define paramters for simulated transfer function for Left Wheel
k_2 = (9.36) / 4;
sigma_2 = 14.81;
sys_2 = tf(k_2*sigma_2, [1 sigma_2]);
step(sys_2 * 4, 'b');
title("Open Step Response");
xlabel('Time');
ylabel('Velocity (rad/s)');


%% Loop step response
%
% This simulation incorporates a PI controller. This controller can be
% tuned in Simulink to ensure Rise times of less than 1 seconds that have
% less than 10% overshoot.

% Open Closed Loop step response simulation
open_system('RunRhoStepExperiment')
%
% Run the simulation
%
out=sim('RunRhoStepExperiment');

% Plot the Results for both Wheels

% Right Wheel
plot(out.position1)
title('Position');
% Left Wheel
plot(out.position)
title('Position');