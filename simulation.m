% Main Script to simulate the zigzag laser motion and plot in 3D
clear;
clc;

% Simulation parameters
max_radius = 5;
speed = 100; % Speed of the zigzag pattern
field_speed = 10; % Speed of the field center movement
total_time = 600; % total simulation time in seconds
dt = 0.0001; % smaller time step for smoother simulation
fps = 30; % Frames per second for playback
update_interval = 1 / fps; % update the plot every 1/fps seconds

% Field parameters
field_size = 160;
field_center = [0, 0]; % Initial center of the field
initial_direction = [10, 3]; % Initial direction of the field movement

% Random factor for bouncing
random_factor = 0.5; % Random factor for the bouncing behavior

% Plot parameters
plot_size = [-100 100 -100 100 -100 100]; % Size of the plot [xmin xmax ymin ymax zmin zmax]
marker_size = 10; % Size of the markers

% Zigzag parameters
zigzag_dist = 100; % Distance from the laser origin

% Precompute the simulation data
num_steps = floor(total_time / dt) + 1;
positions = zeros(num_steps, 3); % to store [x, z, y] positions (y and z swapped)
times = zeros(num_steps, 1); % to store time steps
field_positions = zeros(num_steps, 2); % to store field center positions

% Main simulation loop to calculate positions
index = 1;
for t = 0:dt:total_time
    % Get the field center position
    [field_x, field_y] = field_motion(t, field_size, field_speed, field_center, initial_direction, random_factor);
    field_positions(index, :) = [field_x, field_y];
    
    % Calculate zigzag position relative to the field center
    [x, z] = zigzag_laser(t, max_radius, speed, [field_x, field_y]);
    [theta, phi, x_end, y_end, z_end] = laser_control(x, z, zigzag_dist);
    
    % Store the computed positions and time
    positions(index, :) = [x_end, z_end, y_end]; % Note the swap of y and z
    times(index) = t;
    index = index + 1;
end

% Animation loop to update the plot
num_frames = floor(total_time / update_interval);
x_vals = [];
y_vals = [];
z_vals = [];
for frame = 1:num_frames
    % Calculate the corresponding index in the precomputed positions
    t = (frame - 1) * update_interval;
    [~, idx] = min(abs(times - t));
    x_end = positions(idx, 1);
    z_end = positions(idx, 2);
    y_end = positions(idx, 3); % Note the swap of y and z
    
    % Store the positions for scatter plot
    x_vals = [x_vals, x_end];
    y_vals = [y_vals, y_end];
    z_vals = [z_vals, z_end];
    
    % Plot the laser in 3D space
    plot3D(x_end, y_end, z_end, x_vals, y_vals, z_vals, plot_size, marker_size); % Note the order of arguments
    pause(update_interval);
end

function [x, z] = zigzag_laser(t, max_radius, speed, field_center)
    % Function to generate a smooth zigzag motion for the laser within a circle
    % Inputs:
    %   t: current time (scalar)
    %   max_radius: maximum radius of the circle (scalar)
    %   speed: speed of the laser motion (scalar)
    %   field_center: [x, y] coordinates of the field center (1x2 vector)
    % Outputs:
    %   x: x position of the laser (scalar)
    %   z: z position of the laser (scalar)

    % Parameters
    persistent current_position target_position last_update_time;

    if isempty(current_position)
        % Initialize the current and target positions relative to the field center
        current_position = field_center;
        target_position = getBoundaryPosition(max_radius) + field_center;
        last_update_time = t;
    end

    % Calculate the time step
    dt = t - last_update_time;

    % Calculate the direction vector towards the target position
    direction_vector = target_position - current_position;

    % Calculate the distance to the target position
    distance_to_target = norm(direction_vector);

    % Move towards the target position
    if distance_to_target > speed * dt
        % Normalize the direction vector and move towards the target
        direction_vector = direction_vector / distance_to_target;
        current_position = current_position + direction_vector * speed * dt;
    else
        % If the target is reached, select a new random target position on the boundary
        current_position = target_position;
        target_position = getBoundaryPosition(max_radius) + field_center;
    end

    % Update the last update time
    last_update_time = t;

    % Output the current position
    x = current_position(1);
    z = current_position(2);
end

function [field_x, field_y] = field_motion(t, field_size, field_speed, field_center, initial_direction, random_factor)
    % Function to calculate the motion of the field center within a 5x5 field
    % Inputs:
    %   t: current time (scalar)
    %   field_size: size of the field (scalar)
    %   field_speed: speed of the field center movement (scalar)
    %   field_center: initial center of the field [x, y] (1x2 vector)
    %   initial_direction: initial direction of the field movement [dx, dy] (1x2 vector)
    %   random_factor: factor influencing the randomness of the bounce
    % Outputs:
    %   field_x, field_y: coordinates of the field center (scalars)

    % Normalize the initial direction
    initial_direction = initial_direction / norm(initial_direction);

    % Calculate the field boundaries
    half_size = field_size / 2;
    min_bound = field_center - half_size;
    max_bound = field_center + half_size;
    
    % Initialize the field position and velocity
    persistent field_position field_velocity last_update_time;
    if isempty(field_position)
        field_position = field_center;
        field_velocity = field_speed * initial_direction; % Initial velocity
        last_update_time = t;
    end

    % Calculate the time step
    dt = t - last_update_time;

    % Update the field position
    field_position = field_position + field_velocity * dt;

    % Check for boundary collisions and reverse velocity with random factor if necessary
    for i = 1:2
        if field_position(i) <= min_bound(i) || field_position(i) >= max_bound(i)
            field_velocity(i) = -field_velocity(i) + (rand - 0.5) * random_factor;
            field_position(i) = max(min(field_position(i), max_bound(i)), min_bound(i));
        end
    end

    % Update the last update time
    last_update_time = t;

    % Output the field position
    field_x = field_position(1);
    field_y = field_position(2);
end

function position = getBoundaryPosition(max_radius)
    % Generate a random position on the boundary of the circle of given max_radius
    angle = rand * 2 * pi;
    position = [max_radius * cos(angle), max_radius * sin(angle)];
end

function [theta, phi, x_end, y_end, z_end] = laser_control(x, z, dist)
    % Function to control the laser in a 3D space
    % Inputs:
    %   x: x position of the target point in the plane
    %   z: z position of the target point in the plane
    %   dist: distance of the plane from the laser origin
    % Outputs:
    %   theta: angle in the xy-plane (azimuth angle)
    %   phi: angle from the z-axis (elevation angle)
    %   x_end, y_end, z_end: coordinates of the end point of the laser

    % Laser origin
    x_origin = 0;
    y_origin = 0;
    z_origin = 0;

    % End point coordinates
    x_end = x;
    z_end = z;
    y_end = dist;

    % Calculate distances
    d_xy = sqrt(x^2 + z^2);
    d_total = sqrt(d_xy^2 + dist^2);

    % Calculate angles
    theta = atan2(z, x);  % Azimuth angle in the xy-plane
    phi = acos(dist / d_total);  % Elevation angle from the z-axis
end

function plot3D(x_end, y_end, z_end, x_vals, y_vals, z_vals, plot_size, marker_size)
% Function to plot the laser in 3D space within a configurable volume
% Inputs:
%   x_end, y_end, z_end: coordinates of the end point of the laser
%   x_vals, y_vals, z_vals: arrays of end point coordinates
%   plot_size: size of the plot [xmin xmax ymin ymax zmin zmax] (1x6 vector)
%   marker_size: size of the markers

persistent h scatter_handle plot_initialized;
if isempty(plot_initialized)
    figure;
    h = plot3([0, x_end], [0, y_end], [0, z_end], 'r-', 'LineWidth', 2);
    hold on;
    scatter_handle = scatter3(x_vals, y_vals, z_vals, marker_size, 'bo', 'filled');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    axis(plot_size);
    axis equal;
    plot_initialized = true;
else
    set(h, 'XData', [0, x_end], 'YData', [0, y_end], 'ZData', [0, z_end]);
    set(scatter_handle, 'XData', x_vals, 'YData', y_vals, 'ZData', z_vals, 'SizeData', marker_size);
    axis(plot_size);
    drawnow;
end
end

