%% Prediction and Update Step
variance_p = 0.01;
variance_v = 0.001;

posterior_x_k = zeros(length(x_k),1);
posterior_x_k_covariance = diag([variance_p,variance_v,variance_p,variance_v]);
posterior_x_k_px_std_list = zeros(1,length(time));
posterior_x_k_py_std_list = zeros(1,length(time));

Q_variance_matrix = diag([0,Q_variance*5,0,Q_variance]);
R_variance_matrix = diag([R_variance,R_variance]);

priori_x_k_covariance =  zeros(length(x_k));
priori_x_k = zeros(length(x_k),1);



for k = 1:length(time)
    priori_x_k = F*posterior_x_k + G*u_k;
    priori_x_k_covariance = F*posterior_x_k_covariance*F' + Q_variance_matrix;

    y_k =  z_k_log(:,k) - H*priori_x_k;
    s_k = H*priori_x_k_covariance*H' + R_variance_matrix;
    k_k = (priori_x_k_covariance*H')/s_k;
    posterior_x_k = priori_x_k + k_k*y_k;
    posterior_x_k_covariance = (eye(4) - k_k*H)*priori_x_k_covariance;

    posterior_x_k_px_list(k) = posterior_x_k(1);
    posterior_x_k_px_std_list(k) = sqrt(posterior_x_k_covariance(1,1));

    posterior_x_k_py_list(k) = posterior_x_k(3);
    posterior_x_k_py_std_list(k) = sqrt(posterior_x_k_covariance(3,3));


end

figure

subplot(2, 2, 1)
plot(time, posterior_x_k_px_list)
title("posterior_px")
ylabel("posterior_px (m)")
xlabel("time (s)")

subplot(2, 2, 2)
plot(time, posterior_x_k_px_std_list)
title("posterior_x_k_px_std_list")
ylabel("posterior_x_k_px_std_list (m)")
xlabel("time (s)")

subplot(2, 2, 3)
plot(time, posterior_x_k_py_list)
title("posterior_x_k_py")
ylabel("posterior_x_k_py (m)")
xlabel("time (s)")

subplot(2, 2, 4)
plot(time, posterior_x_k_py_std_list)
title("posterior_x_k_py_std")
ylabel("posterior_x_k_py_std (m)")
xlabel("time (s)")

sgtitle("LKF on the car")

%% LKF Tracking Metrics

rmse_lkf_px = sqrt(mean((px - posterior_x_k_px_list).^2));
rmse_lkf_py = sqrt(mean((py - posterior_x_k_py_list).^2));

% Sensor tracking error

rmse_sensor_px = sqrt(mean((px - z_k_log(1,:)).^2));
rmse_sensor_py = sqrt(mean((px - z_k_log(2,:)).^2));

fprintf('\n================ FILTER BENCHMARK ================\n');
fprintf('Position X Error -> Sensor: %.4f m | Kalman: %.4f m\n', rmse_sensor_px, rmse_lkf_px);
fprintf('Position Y Error -> Sensor: %.4f m | Kalman: %.4f m\n', rmse_sensor_py, rmse_lkf_py);
fprintf('==================================================\n');

%% Birds Eye View

%% 2D Trajectory Map Visualization
figure('Name', '2D Vehicle Trajectory Tracking', 'NumberTitle', 'off')

% 1. Plot the raw, noisy sensor data points
plot(z_k_log(1,:), z_k_log(2,:), 'r.', 'MarkerSize', 5, 'DisplayName', 'Noisy Sensor Data'); 
hold on;

% 2. Plot the true simulated trajectory line
plot(px, py, 'g-', 'LineWidth', 2.5, 'DisplayName', 'True Trajectory');

% 3. Plot the optimized linear Kalman Filter estimate
plot(posterior_x_k_px_list, posterior_x_k_py_list, 'b--', 'LineWidth', 2, 'DisplayName', 'LKF Path Estimate');

% Formatting the map layout
title('2D Position Tracking Map (Bird''s-Eye View)', 'FontSize', 12)
xlabel('Position X (m)', 'FontSize', 10)
ylabel('Position Y (m)', 'FontSize', 10)
grid on
axis equal  % Crucial: ensures 1 meter on X-axis equals 1 meter on Y-axis
legend('Location', 'best', 'FontSize', 10)
