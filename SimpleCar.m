
%% Clear 
clc
clear
%% Declaration of state variables

delta_t = 0.01;
simulation_time = 10;
time = 0:delta_t:simulation_time;

F = [1 delta_t 0 0; 0 1 0 0; 0 0 1 delta_t; 0 0 0 1];
G = [0.5*(delta_t)^2 0; delta_t 0; 0 0.5*(delta_t)^2; 0 delta_t];

H = [1 0 0 0; 0 0 1 0];

genNoise = @(mu,sigma,r,c) mu + sigma*randn(r,c);

Q_variance = 0.001;
R_variance = 0.01;

x_k = [0; 0; 0; 0];
u_k = [0.1; 0];
z_k = zeros(2,1);
z_k_log = zeros(2,length(time));

z_px = zeros(1,length(time));
z_py = zeros(1,length(time));

px = zeros(1,length(time));
vx =zeros(1,length(time));
py = zeros(1,length(time));
vy = zeros(1,length(time));

%% Simulation of the car

for k = 1:length(time)
    x_k = F*x_k +  G*(u_k + genNoise(0,Q_variance,length(u_k),1) ) + [0 0 0 0;0 1 0 0; 0 0 0 0; 0 0 0 1]*genNoise(0,Q_variance,length(x_k),1);
    z_k = H*x_k + genNoise(0,R_variance,2,1);
    z_k_log(:,k) = z_k;
    z_px(k) = z_k(1);
    z_py(k)= z_k(2);

    px(k) = x_k(1);
    vx(k) = x_k(2);
    py(k) = x_k(3);
    vy(k) = x_k(4);
end

figure

subplot(3, 2, 1)
plot(time, px)
title("px")
ylabel("px (m)")
xlabel("time (s)")

subplot(3, 2, 2)
plot(time, py)
title("py")
ylabel("py (m)")
xlabel("time (s)")

subplot(3, 2, 3)
plot(time, z_px)
title("z_px")
ylabel("z_px (m)")
xlabel("time (s)")

subplot(3, 2, 4)
plot(time, z_py)
title("z_py")
ylabel("z_py (m)")
xlabel("time (s)")

subplot(3, 2, 5)
plot(time, vx)
title("vx")
ylabel("vx (m/s)")
xlabel("time (s)")

subplot(3, 2, 6)
plot(time, vy)
title("vy")
ylabel("vy (m/s)")
xlabel("time (s)")

sgtitle("Kinematic State Space of a Car")

