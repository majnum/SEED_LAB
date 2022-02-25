
%% Calculate Controller output
% sample time
Ts = t - t_past;
% Integrator
I = I_past+e*Ts;
% PI control calculation
6
u = Kp*e+Ki*I;
% anti-windup
if abs(u)>umax,
u = sign(u)*umax;
e = sign(e)*min(umax/Kp, abs(e));
I = (u-Kp*e)/Ki;
end;
I_past=I;
t_past=t;
