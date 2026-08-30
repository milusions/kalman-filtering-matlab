clc
clear
%% Variables
F1 = 5.5;
F2 = 5.5;
U = [F1 F2];
M = 1;
L = 0.3;
g = 10;
I = M*L^2/12;
delta_t = 0.01;
end_time = 10;
time = 0:delta_t:end_time;

genNoise = @(mu,sigma_sq,r,c) mu + sqrt(sigma_sq)*randn(r,c);

x_k = zeros(6,1);

Q_variance = 0.001;
noise_selector = zeros(6,3);

noise_selector(2,1) = 1;
noise_selector(4,2) = 1;
noise_selector(6,3) = 1;

posterior_x_k = zeros(6,1);
posterior_x_k_covariance = zeros(6);

priori_x_k = zeros(6,1);
priori_x_k_covariance = zeros(6);

Q_covariance_matrix = diag([0,Q_variance,0,Q_variance,0,Q_variance]);


posterior_y_list = zeros(1,length(time));
posterior_y_covariance_list = zeros(1,length(time));

posterior_x_list = zeros(1,length(time));
posterior_x_covariance_list = zeros(1,length(time));

posterior_theta_list = zeros(1,length(time));
posterior_theta_covariance_list = zeros(1,length(time));


posterior_y_dot_list = zeros(1,length(time));
posterior_y_dot_covariance_list = zeros(1,length(time));

posterior_x_dot_list = zeros(1,length(time));
posterior_x_dot_covariance_list = zeros(1,length(time));

posterior_theta_dot_list = zeros(1,length(time));
posterior_theta_dot_covariance_list = zeros(1,length(time));

x_list = zeros(1,length(time));
y_list = zeros(1,length(time));
theta_list = zeros(1,length(time));

measured_x_list = zeros(1,length(time));
measured_y_list = zeros(1,length(time));

H = [1 0 0 0 0 0; 0 0 1 0 0 0];
R_variance = 0.5;
R = diag([R_variance,R_variance]);
%% EKF Prediction

for k = 1:1:length(time)
    a = (U(1) + U(2))/M;
    alpha = (U(2) - U(1))*L/(2*I);
    F_jacobian = [1 delta_t 0 0 -0.5*a*cos(posterior_x_k(5))*delta_t^2 0; 0 1 0 0 -a*cos(posterior_x_k(5))*delta_t 0; 0 0 1 delta_t -0.5*a*sin(posterior_x_k(5))*delta_t^2 0; 0 0 0 1 -a*sin(posterior_x_k(5))*delta_t 0; 0 0 0 0 1 delta_t;0 0 0 0 0 1];
    
    term_2 = [posterior_x_k(2); -a*sin(posterior_x_k(5)); posterior_x_k(4); (a*cos(posterior_x_k(5)) - g); posterior_x_k(6);alpha];
    term_3 = [-a*sin(posterior_x_k(5)); 0; (a*cos(posterior_x_k(5)) - g); 0; alpha; 0];


    x_k_term_2 = [x_k(2); -a*sin(x_k(5)); x_k(4); (a*cos(x_k(5)) - g); x_k(6);alpha];
    x_k_term_3 = [-a*sin(x_k(5)); 0; (a*cos(x_k(5)) - g); 0; alpha; 0];
    
    
    x_k = x_k + delta_t.*x_k_term_2 + 0.5*(delta_t^2).*x_k_term_3 + noise_selector*genNoise(0,Q_variance,3,1);
    x_k(5) = atan2(sin(x_k(5)), cos(x_k(5)));

    priori_x_k = posterior_x_k + delta_t.*term_2 + 0.5*(delta_t^2).*term_3;
    priori_x_k_covariance = F_jacobian*posterior_x_k_covariance*transpose(F_jacobian) + Q_covariance_matrix;
    priori_x_k(5) = atan2(sin(priori_x_k(5)), cos(priori_x_k(5)));
    
    if 5*randn(1) > 0
    z_k = H*x_k + genNoise(0,R_variance,2,1);

    innovation = z_k - H*priori_x_k;
    innovation_covariance = H*priori_x_k_covariance*transpose(H) + R;
    kalman_gain = priori_x_k_covariance*transpose(H)*inv(innovation_covariance);
    
    posterior_x_k = priori_x_k + kalman_gain*innovation;
    posterior_x_k_covariance = (eye(6) - kalman_gain*H)*priori_x_k_covariance;
    
else

    posterior_x_k = priori_x_k;
    posterior_x_k_covariance = priori_x_k_covariance;
end
    posterior_x_list(k) = posterior_x_k(1);
    posterior_x_covariance_list(k) = posterior_x_k_covariance(1,1);

    posterior_y_list(k) = posterior_x_k(3);
    posterior_y_covariance_list(k) = posterior_x_k_covariance(3,3);

    posterior_theta_list(k) = posterior_x_k(5);
    posterior_theta_covariance_list(k) = posterior_x_k_covariance(5,5);

    posterior_x_dot_list(k) = posterior_x_k(2);
    posterior_x_dot_covariance_list(k) = posterior_x_k_covariance(2,2);

    posterior_y_dot_list(k) = posterior_x_k(4);
    posterior_y_dot_covariance_list(k) = posterior_x_k_covariance(4,4);

    posterior_theta_dot_list(k) = posterior_x_k(6);
    posterior_theta_dot_covariance_list(k) = posterior_x_k_covariance(6,6);

     x_list(k) = x_k(1);
    y_list(k) = x_k(3);
    theta_list(k) = x_k(5);

    measured_x_list(k) = z_k(1);
    measured_y_list(k) = z_k(2);
   
end
%% Plot EKF
figure
subplot(3,2,1)
plot(time,posterior_x_list)
title("Posterior X data")
ylabel("x")
xlabel("time")

subplot(3,2,2)
plot(time,posterior_x_covariance_list)
title("Posterior X Covariance data")
ylabel("x covariance")
xlabel("time")

subplot(3,2,3)
plot(time,posterior_y_list)
title("Posterior Y data")
ylabel("y")
xlabel("time")

subplot(3,2,4)
plot(time,posterior_y_covariance_list)
title("Posterior Y Covariance data")
ylabel("y covariance")
xlabel("time")

subplot(3,2,5)
plot(time,posterior_theta_list)
title("Posterior theta data")
ylabel("theta")
xlabel("time")

subplot(3,2,6)
plot(time,posterior_theta_covariance_list)
title("Posterior theta Covariance data")
ylabel("theta covariance")
xlabel("time")

figure
subplot(3,2,1)
plot(time,posterior_x_dot_list)
title("Posterior X dot data")
ylabel("x_dot")
xlabel("time")

subplot(3,2,2)
plot(time,posterior_x_dot_covariance_list)
title("Posterior X_dot Covariance data")
ylabel("x_dot covariance")
xlabel("time")

subplot(3,2,3)
plot(time,posterior_y_dot_list)
title("Posterior Y_dot data")
ylabel("y")
xlabel("time")

subplot(3,2,4)
plot(time,posterior_y_dot_covariance_list)
title("Posterior Y_dot Covariance data")
ylabel("y_dot covariance")
xlabel("time")

subplot(3,2,5)
plot(time,posterior_theta_dot_list)
title("Posterior theta_dot data")
ylabel("theta_dot")
xlabel("time")

subplot(3,2,6)
plot(time,posterior_theta_dot_covariance_list)
title("Posterior theta_dot Covariance data")
ylabel("theta_dot covariance")
xlabel("time")

%% RMSE

rmse_gps_x = sqrt(mean((measured_x_list - x_list).^2));
rmse_gps_y = sqrt(mean((measured_y_list - y_list).^2));

% RMSE for EKF Estimate vs True State (After EKF filtering)
rmse_ekf_x = sqrt(mean((posterior_x_list - x_list).^2));
rmse_ekf_y = sqrt(mean((posterior_y_list - y_list).^2));

fprintf('\n--- Root Mean Square Error (RMSE) ---\n');
fprintf('X Position - Raw GPS Noise: %.4f m | EKF Estimate: %.4f m\n', rmse_gps_x, rmse_ekf_x);
fprintf('Y Position - Raw GPS Noise: %.4f m | EKF Estimate: %.4f m\n', rmse_gps_y, rmse_ekf_y);

%% plot paths
figure
plot(posterior_x_list,posterior_y_list)
hold on
plot(measured_x_list,measured_y_list,"r.")
plot(x_list,y_list)
title("Paths")
ylabel("x")
legend("ekf","gps","true")
xlabel("y")