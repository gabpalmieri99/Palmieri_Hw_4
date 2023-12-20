bag = rosbag('robot_traj.bag');

% Select topics
xTopic = '/robot_trajectory_x';
yTopic = '/robot_trajectory_y';

xData = readMessages(select(bag, 'Topic', xTopic));
yData = readMessages(select(bag, 'Topic', yTopic));

xValues = cellfun(@(msg) msg.Data, xData);
yValues = cellfun(@(msg) msg.Data, yData);

nPoints = length(xValues);
colors = linspace(0, 1, nPoints);

% Plot
figure;
scatter(xValues, yValues, 50, colors, 'filled');

hold on;

fixedPointsX = [-3, -6, -17.5, -15, -10];
fixedPointsY = [5, 8, 3, 7, 3];
scatter(fixedPointsX, fixedPointsY, 100, 'r', 'Marker', 'x', 'LineWidth', 2);

% Labels
labels = {'homepos', 'goal4', 'goal3', 'goal2', 'goal1'};
for i = 1:length(labels)
    text(fixedPointsX(i), fixedPointsY(i), labels{i}, 'Color', 'r', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

title('Robot Trajectory');
xlabel('x[m]');
ylabel('y[m]');
colorbar; 
grid on;

figure;
subplot(2, 1, 1);
plot(xValues);
title('x Trajectory');
xlabel('Time');
ylabel('x [m]');
grid on;

subplot(2, 1, 2);
plot(yValues, 'r');
title('y Trajectory');
xlabel('Time');
ylabel('y [m]');
grid on;
