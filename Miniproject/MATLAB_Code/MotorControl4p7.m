
K = 7.85;
sig = 50;
s = tf('s');

open_system('PIControl')
%
% Run the simulation
%
out=sim('PIControl');
%% A Plot of the results: Velocity and Postion
%
% We see that the model transfer function is most similar to the transfer 
% function of the motor with inertial load model when sigma is tuned to 
% 10.0 and K is tuned to 0.67. These graphs show the velocity and position 
% outputs of each model.The velocity and position were found using
% integrator and derivative blocks as needed keeping in mind that velocity 
% is the derivative of position.
%


%Position
figure
plot(out.Position)
title('Position')
ylabel('Radians')