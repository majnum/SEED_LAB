%% Controll Simulation for Theta Desired (Outer Loop)
% Group Team 9 
%
% This script runs a simulation of the robot (Both wheels) with a PI
% controller. This controller has been tuned to have a rise time of under 1
% second and to have less than 10% overshoot
%
% Required File: RunDemo2OuterControl.slx
%
%% Define Motor Paramters
Ra=1;  % armaature resistance [Ohms]
Kt=.5; % motor torque constant [Nm/A]
Ke=.5; % back emf constant [Vs/rad]
% J=.05; % Load inertia [Nm^2]           NOT USED -  
% b=.5;  % damping [Nm/s]                NOT USED - 

% Manual Tuning Paramaters
k = 0.39;
sigma = 3.3;

%% Run a simulation 
%
% This simulation applies a PI controller to the transfer function found in
% RunFeedbackControl and the PI can be tuned to lower the overshoot
%
%
% Open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('RunFinalDemo1ControlCode')
%
% run the simulation
%
out=sim('RunFinalDemo1ControlCode');

%% Plot the results 

% We see that the rise time and overshoot requirments are met. Manual
% tuning was required to get edgar to work. 

plot(out.position1)
title('Position1');