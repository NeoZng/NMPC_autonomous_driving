function [Ref, End, Obs, radius] = loadMap(horizon)
% Plotting the ring road and obstacles
load('ring_road.mat');
figure;
hold on;
plot(outer_boundary_x, outer_boundary_y, 'r', 'LineWidth', 2); % Outer boundary
plot(inner_boundary_x, inner_boundary_y, 'r', 'LineWidth', 2); % Inner boundary
plot(center_line_x, center_line_y, 'k--', 'LineWidth', 1.5);   % Center line

% Plot obstacles
for i = 1:length(obstacle_positions)
    viscircles(obstacle_positions(i, :), obstacle_radius, 'Color', 'b');
end
axis equal;
title('Ring Road with Obstacles');
xlabel('X (m)');
ylabel('Y (m)');
legend('Outer Boundary', 'Inner Boundary', 'Center Line', 'Obstacles');
hold off;

Ref = [center_line_x,center_line_x; center_line_y,center_line_y];
End = length(Ref(1,:))-horizon;
Obs = obstacle_positions.';
radius = obstacle_radius;
end