clear;
clc;
close;

% Set the time step for your simulation
time_step = 0.1;
num_wheels = 4;
% Call the animation function and get the contact points for each frame
all_contact_points = contact_points_animation_from_stl('wheel.stl');
num_frames = numel(all_contact_points);
% Add the wheel positions to the code
wheel_positions = [0.166, 0.2924, -0.052103; ...
                   -0.166, 0.2924, -0.052103; ...
                   0.166, -0.2924, -0.052103; ...
                   -0.166, -0.2924, -0.052103];
% Calculate velocities, accelerations, and ICR for each frame
[linear_velocities, angular_velocities, linear_accelerations, angular_accelerations, icr_points, wheel_velocity] = ...
    calculate_wheel_properties(all_contact_points, num_frames, time_step, num_wheels);

% Calculate the robot's forward and angular velocities
[robot_forward_velocity, robot_angular_velocity] = calculate_robot_velocities(num_frames, num_wheels, linear_velocities, wheel_positions);


% Calculate the robot's position and orientation (yaw)
[robot_position, robot_yaw] = calculate_robot_pose(num_frames, robot_forward_velocity, robot_angular_velocity, time_step);

% Plot the robot's trajectory
plot_robot_trajectory(robot_position, robot_yaw);

% ---- Functions ----

function [all_contact_points] = contact_points_animation_from_stl(stl_file)
      % Import the STL file
    fv = stlread(stl_file);
    scaled_points = fv.Points / 1000;
    % Animation parameters
    duration = 20;
    num_frames = 100;
    time_step = duration / num_frames;
    
    % Set up the figure for the animation
    figure;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Contact Points Animation from STL File');
    view(3); % 3D view
    hold on;
    camlight('headlight');
    material('shiny');

    % Initialize the cell array to store contact points for each frame
    all_contact_points = cell(num_frames, 1);

    % Loop through each frame of the animation
    for frame = 1:num_frames
        % Rotate the wheel by a small angle for each frame
        angle_rad = 2 * pi * (frame - 1) / num_frames;
        rotation_matrix = [cos(angle_rad), 0, sin(angle_rad); ...
                           0, 1, 0; ...
                           -sin(angle_rad), 0, cos(angle_rad)];
        rotated_vertices = scaled_points * rotation_matrix';
        
        % Calculate the contact points for the current frame
        contact_points = calculate_contact_points(rotated_vertices);
        all_contact_points{frame} = contact_points; % Store the contact points for the current frame

        % Plot the rotated wheel and contact points
        wheel_plot = patch('Faces', fv.ConnectivityList, 'Vertices', rotated_vertices, ...
                           'FaceColor', [0.8 0.8 1.0], ...
                           'EdgeColor', 'none', ...
                           'FaceLighting', 'gouraud', ...
                           'AmbientStrength', 0.15);
        contact_plot = scatter3(contact_points(:,1), contact_points(:,2), contact_points(:,3), ...
                                'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75],'LineWidth',1.5);

        % Pause for the time step and delete the plots to prepare for the next frame
        pause(time_step);
        if frame < num_frames
            delete(wheel_plot);
            delete(contact_plot);
        end
    end
end

function contact_points = calculate_contact_points(vertices)
     % Find the distance from each vertex to the ground (X-Y plane)
    distances_to_ground = vertices(:,3);
    
    % Get the minimum distance to the ground
    min_distance = min(distances_to_ground);
    
    % Use a tolerance and a small Z-offset to find the vertices that are in contact with the ground
    tolerance = 1e-5;
    z_offset = 0.001;
    contact_indices = abs(distances_to_ground - (min_distance)) < tolerance;

    % Filter contact points based on Y-axis
    filtered_vertices = vertices(contact_indices, :);
    y_offset = 100;
    contact_indices_y = filtered_vertices(:,2) < (max(filtered_vertices(:,2)) + y_offset);
    contact_points = filtered_vertices(contact_indices_y, :);
end

function [linear_velocity, angular_velocity, icr] = calculate_velocities_and_icr(contact_points)
     % Define the wheel's angular velocity (in radians per second)
    angular_speed = 10*2 * pi; % Example: 1 revolution per second
    angular_velocity = [0, angular_speed, 0]; % Assuming rotation around the Y-axis

    % Calculate the linear velocity of each contact point
    linear_velocity = zeros(size(contact_points));
    for i = 1:size(contact_points, 1)
        linear_velocity(i, :) = cross(angular_velocity, contact_points(i, :));
    end
    
    % Calculate the ICR (Instantaneous Center of Rotation)
    % For this example, we'll calculate the average position of the contact points
    icr = mean(contact_points);
end

function [linear_velocities, angular_velocities, linear_accelerations, angular_accelerations, icr_points, wheel_velocity] = ...
        calculate_wheel_properties(all_contact_points, num_frames, time_step, num_wheels)
    % Preallocate arrays
    linear_velocities = cell(num_frames, 1);
    angular_velocities = cell(num_frames, 1);
    linear_accelerations = cell(num_frames, 1);
    angular_accelerations = cell(num_frames, 1);
    icr_points = cell(num_frames, 1);
    wheel_velocity = zeros(num_frames, 3);

    % Calculate velocities, accelerations, and ICR for each frame
    for frame = 1:num_frames
        contact_points = all_contact_points{frame};
        [linear_velocity, angular_velocity, icr] = calculate_velocities_and_icr(contact_points);

        % Duplicate the velocities for each wheel
        linear_velocity_all_wheels = repmat(linear_velocity, num_wheels / size(linear_velocity, 1), 1);

        % Store the linear and angular velocities
        linear_velocities{frame} = linear_velocity_all_wheels;
        angular_velocities{frame} = angular_velocity;

        % Calculate the wheel velocity as the mean of the contact points linear velocities
        wheel_velocity(frame, :) = mean(linear_velocity_all_wheels, 1);

        % Calculate the accelerations for the current frame
        if frame > 1
            linear_accelerations{frame} = (linear_velocity_all_wheels - linear_velocities{frame-1}) / time_step;
            angular_accelerations{frame} = (angular_velocity - angular_velocities{frame-1}) / time_step;
        end

        % Store the ICR
        icr_points{frame} = icr;
    end
end

function [robot_forward_velocity, robot_angular_velocity] = calculate_robot_velocities(num_frames, num_wheels, linear_velocities, wheel_positions)
    % Calculate the forward linear velocities of the wheels in the robot's frame
    forward_velocities_wheels = zeros(num_frames, num_wheels);

    for frame = 1:num_frames
        for wheel = 1:num_wheels
            wheel_direction = [cos(pi*(wheel-1)/2), sin(pi*(wheel-1)/2)];
            forward_velocities_wheels(frame, wheel) = dot(linear_velocities{frame}(wheel, [1, 3]), wheel_direction);
        end
    end

    % Calculate the robot's forward and angular velocities
    robot_forward_velocity = mean(forward_velocities_wheels, 2);
    
    % Calculate the distances of the wheels to the center of the robot
    wheel_distances_to_center = sqrt(sum(wheel_positions(:, [1, 2]) .^ 2, 2));
    
    % Calculate the robot's angular velocity based on the forward velocities of the wheels and their distances to the center
    robot_angular_velocity = (-forward_velocities_wheels(:, 1) - forward_velocities_wheels(:, 2) + forward_velocities_wheels(:, 3) + forward_velocities_wheels(:, 4)) / 0.664;
%     robot_angular_velocity= zeros(num_frames, 1); % Set angular velocity to zero for straight trajectory
end


function [robot_position, robot_yaw] = calculate_robot_pose(num_frames, robot_forward_velocity, robot_angular_velocity, time_step)
% Initialize the robot's position and orientation (yaw)
robot_position = zeros(num_frames, 2);
robot_yaw = zeros(num_frames, 1);

% Calculate the robot's position and orientation (yaw) for each frame
for frame = 2:num_frames
    robot_yaw(frame) = robot_yaw(frame - 1) + robot_angular_velocity(frame - 1) * time_step;
    robot_position(frame, :) = robot_position(frame - 1, :) + ...
        robot_forward_velocity(frame - 1) * time_step * [cos(robot_yaw(frame - 1)), sin(robot_yaw(frame - 1))];
end
end

function plot_robot_trajectory(robot_position, robot_yaw)
% Plot the robot's trajectory
figure;
plot(robot_position(:, 1), robot_position(:, 2), 'b-', 'LineWidth', 2);
hold on;
% Plot the robot's orientation (yaw) as arrows
for i = 1:10:length(robot_yaw)
    quiver(robot_position(i, 1), robot_position(i, 2), cos(robot_yaw(i)), sin(robot_yaw(i)), 'r');
end
hold off;

% Set plot properties
xlabel('X [m]');
ylabel('Y [m]');
title('Robot Trajectory');
legend('Trajectory', 'Orientationofrbt');
grid on;
axis equal;
end
