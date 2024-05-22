% Parameters
radius = 20;      % Radius from origin to the middle of the road (m)
road_width = 8;   % Width of the road (m)
num_obstacles = 7; % Number of obstacles
obstacle_radius = 1; % Radius of each obstacle (m)

% Ring road boundaries
theta = linspace(0, 2*pi, 400);
outer_boundary_x = (radius + road_width/2) * cos(theta);
outer_boundary_y = (radius + road_width/2) * sin(theta);
inner_boundary_x = (radius - road_width/2) * cos(theta);
inner_boundary_y = (radius - road_width/2) * sin(theta);
center_line_x = radius * cos(theta);
center_line_y = radius * sin(theta);

% Randomly generate obstacles on the ring road
obstacle_positions = [];
for i = 1:num_obstacles
    % equally distributed obstacles along the angle
    angle = 2 * pi * i / num_obstacles + pi/2;
    dist_from_center = radius - road_width/4 + road_width/2 * rand();
    obstacle_x = dist_from_center * cos(angle);
    obstacle_y = dist_from_center * sin(angle);
    obstacle_positions = [obstacle_positions; obstacle_x, obstacle_y];
end

% Save the boundaries and obstacles
save('ring_road.mat', 'outer_boundary_x', 'outer_boundary_y', ...
                     'inner_boundary_x', 'inner_boundary_y', ...
                     'center_line_x', 'center_line_y', ...
                     'obstacle_positions','obstacle_radius',"num_obstacles");

% Plotting the ring road and obstacles
figure;
hold on;
plot(outer_boundary_x, outer_boundary_y, 'r', 'LineWidth', 2); % Outer boundary
plot(inner_boundary_x, inner_boundary_y, 'r', 'LineWidth', 2); % Inner boundary
plot(center_line_x, center_line_y, 'k--', 'LineWidth', 1.5);   % Center line

% Plot obstacles
for i = 1:num_obstacles
    viscircles(obstacle_positions(i, :), obstacle_radius, 'Color', 'b');
end

% Plot settings
axis equal;
title('Ring Road with Obstacles');
xlabel('X (m)');
ylabel('Y (m)');
legend('Outer Boundary', 'Inner Boundary', 'Center Line', 'Obstacles');
hold off;
