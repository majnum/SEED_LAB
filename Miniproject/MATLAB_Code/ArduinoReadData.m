Data = csvread('arduinoData.csv',1,0);
ThetaDot = Data (:,1)
time = Data (:,2)

K = 7.85;
sig = 50;
s = tf('s');
figure(1);
hold on;
plot(time, ThetaDot);

G = 7.85 * (sig)/(s+sig)
step(G,1.03);
stepinfo(G)
hold off;

open_system("PIControl");
%
% run the simulation
%
out=sim("PIControl");

figure(2);
plot(out.simout);