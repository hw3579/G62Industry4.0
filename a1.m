% Prepare figure
figure;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Reachable Workspace');
view(3);
grid on;

% Joint angle and position limits
joint1_limits = linspace(-5 * (pi/180), 5 * (pi/180), 10);
joint3_limits = linspace(-30 * (pi/180), 30 * (pi/180), 10);
joint2_limits = linspace(-0.5, 0.5, 10);
joint4_limits = linspace(-1, 1, 10);

% Iterate through joint configurations
for joint1 = joint1_limits
    for joint3 = joint3_limits
        for joint2 = joint2_limits
            for joint4 = joint4_limits
                % Set joint configuration
                config = [joint1, joint2, joint3, joint4]';
               
                
                % Compute forward kinematics
                T = getTransform(robot, config, 'tool');
                
                % Extract end-effector position
            x = T(1, 4);
            y = T(2, 4);
            z = T(3, 4);

            % Plot end-effector position
            plot3(x, y, z, 'r.', 'MarkerSize', 2);

        end
    end
end
end

% Adjust plot view
axis equal;
hold off;