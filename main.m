% Define the initial state of the car
x = 0;
y = 0;
vx = 0;
vy = 0;
ax = 0;
ay = 0;

% Define the target trajectory for the car
target_trajectory = [0 0; 10 5; 20 10; 30 5; 40 0];

% Define the control inputs for the car
throttle = 0.2;
steering_angle = 0.1;

% Set the simulation time step and simulation time
dt = 0.1;
t_end = 10;
t_array = 0:dt:t_end;

% Initialize the arrays to store the car state
x_array = zeros(size(t_array));
y_array = zeros(size(t_array));
vx_array = zeros(size(t_array));
vy_array = zeros(size(t_array));
ax_array = zeros(size(t_array));
ay_array = zeros(size(t_array));

% Loop through the simulation time and update the car state
for i = 1:length(t_array)
    % Calculate the current position error and heading error
    pos_error = norm([x y] - target_trajectory(i,:));
    heading_error = atan2(target_trajectory(i,2) - y, target_trajectory(i,1) - x) - atan2(vy, vx);

    % Calculate the control inputs using a simple proportional controller
    throttle_cmd = throttle;
    steering_angle_cmd = heading_error * 0.1;

    % Update the car state using the dynamics equations
    ax = throttle_cmd * cos(steering_angle) - 0.1 * vx;
    ay = throttle_cmd * sin(steering_angle);
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    x = x + vx * dt;
    y = y + vy * dt;

    % Store the car state in the arrays
    x_array(i) = x;
    y_array(i) = y;
    vx_array(i) = vx;
    vy_array(i) = vy;
    ax_array(i) = ax;
    ay_array(i) = ay;
end

% Plot the longitudinal and lateral acceleration, velocity, and position of the car
figure;
subplot(2,2,1);
plot(t_array, ax_array);
xlabel('Time (s)');
ylabel('Longitudinal acceleration (m/s^2)');

subplot(2,2,2);
plot(t_array, ay_array);
xlabel('Time (s)');
ylabel('Lateral acceleration (m/s^2)');

subplot(2,2,3);
plot(x_array, y_array);
xlabel('X (m)');
ylabel('Y (m)');

subplot(2,2,4);
plot(t_array, vx_array);
hold on;
plot(t_array, vy_array);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Longitudinal velocity', 'Lateral velocity');
