%% plots comparison

% Define URDF file path and import robot model
urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);
chosen_dataset = new_cell_array
output_cell_array = {}

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1, 1, 1, 1, 1, 1]; % IK weights
initialGuess = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector
dampingFactor = 0.5; % Damping factor for IK
maxDeltaAngle = 0.09; % Maximum allowable change in joint angles (radians)

% Initialize storage for comparison data
desiredPositions = []; % To store desired end-effector positions
actualPositions = [];  % To store actual end-effector positions from FK

% Assuming chosen_dataset is already defined as in your original code
for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i};
    data = data(:, 1:20:end); % downsample
    desiredPositions = []; % To store desired end-effector positions
    actualPositions = [];  % To store actual end-effector positions from FK
    desiredPositions_resp = []; % To store desired end-effector positions
    actualPositions_resp = [];  % To store actual end-effector positions from FK
    for t = 1:length(data)
        % --------- First Dataset (chosen_dataset) ---------
        % Command datapoint
        datapoint_command = [data(1, t), data(2, t), data(3, t)];
        
        targetOrientation = eye(3); 
        desiredPose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
        % Update initial guess for IK
        if t > 1
            initialGuess = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [jointAngles, ~] = ik(endEffector, desiredPose, weights, initialGuess);
        
        % Apply damping to the joint movements (if t > 1)
        if t > 1
            dampedJointAngles = dampingFactor * jointAngles(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = dampedJointAngles - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            jointAngles(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store IK results
        j1_com(t, 1) = jointAngles(1);
        j2_com(t, 1) = jointAngles(2);
        j3_com(t, 1) = jointAngles(3);
        j4_com(t, 1) = jointAngles(4);
        j5_com(t, 1) = jointAngles(5);
        o6 = jointAngles(6); o7 = jointAngles(7); o8 = jointAngles(8); o9 = jointAngles(9); o10 = jointAngles(10); o11 = jointAngles(11);
        
        % Store desired position for comparison
        desiredPositions = [desiredPositions; datapoint_command];
        
        % Forward Kinematics using joint angles from IK
        tform = getTransform(robot, jointAngles, endEffector);
        actualPosition = tform2trvec(tform);
        
        % Store actual FK position for comparison
        actualPositions = [actualPositions; actualPosition];








    datapoint_response = [data(4, t), data(5, t), data(6, t)];
            targetOrientation = eye(3); 
        desiredPose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);
        
        % Update initial guess for IK
        if t > 1
            initialGuess = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [jointAngles, ~] = ik(endEffector, desiredPose, weights, initialGuess);
        
        % Apply damping to the joint movements (if t > 1)
        if t > 1
            dampedJointAngles = dampingFactor * jointAngles(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = dampedJointAngles - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            jointAngles(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store IK results
        j1_com(t, 1) = jointAngles(1);
        j2_com(t, 1) = jointAngles(2);
        j3_com(t, 1) = jointAngles(3);
        j4_com(t, 1) = jointAngles(4);
        j5_com(t, 1) = jointAngles(5);
        o6 = jointAngles(6); o7 = jointAngles(7); o8 = jointAngles(8); o9 = jointAngles(9); o10 = jointAngles(10); o11 = jointAngles(11);
        
        % Store desired position for comparison
        desiredPositions_resp = [desiredPositions_resp; datapoint_response];
        
        % Forward Kinematics using joint angles from IK
        tform = getTransform(robot, jointAngles, endEffector);
        actualPosition_resp = tform2trvec(tform);
        
        % Store actual FK position for comparison
        actualPositions_resp = [actualPositions_resp; actualPosition_resp];

        output_cell_array{i} = 0
    end
    % Plotting the results for comparison
% figure;
% 
% % Subplot for X Coordinate
% subplot(3, 1, 1);
% plot(desiredPositions(:, 1), 'r', 'DisplayName', 'Desired X');
% hold on;
% plot(actualPositions(:, 1), 'b', 'DisplayName', 'Actual X');
% title('X Coordinate Comparison');
% xlabel('Time Step');
% ylabel('X Position');
% legend;
% hold off;
% 
% % Subplot for Y Coordinate
% subplot(3, 1, 2);
% plot(desiredPositions(:, 2), 'r', 'DisplayName', 'Desired Y');
% hold on;
% plot(actualPositions(:, 2), 'b', 'DisplayName', 'Actual Y');
% title('Y Coordinate Comparison');
% xlabel('Time Step');
% ylabel('Y Position');
% legend;
% hold off;
% 
% % Subplot for Z Coordinate
% subplot(3, 1, 3);
% plot(desiredPositions(:, 3), 'r', 'DisplayName', 'Desired Z');
% hold on;
% plot(actualPositions(:, 3), 'b', 'DisplayName', 'Actual Z');
% title('Z Coordinate Comparison');
% xlabel('Time Step');
% ylabel('Z Position');
% legend;
% hold off;

end

%% builds output 

% Define URDF file path and import robot model
urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);
chosen_dataset = new_cell_array;
output_cell_array = {};

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1, 1, 1, 1, 1, 1]; % IK weights
initialGuess = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector
dampingFactor = 0.5; % Damping factor for IK
maxDeltaAngle = 0.09; % Maximum allowable change in joint angles (radians)

% Assuming chosen_dataset is already defined as in your original code
for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i};
    data = data(:, 1:20:end); % downsample
    
    desiredPositions = []; % To store desired end-effector positions
    actualPositions = [];  % To store actual end-effector positions from FK
    desiredPositions_resp = []; % To store desired end-effector positions
    actualPositions_resp = [];  % To store actual end-effector positions from FK

    for t = 1:length(data)
        % --------- First Dataset (chosen_dataset) ---------
        % Command datapoint
        datapoint_command = [data(1, t), data(2, t), data(3, t)];
        targetOrientation = eye(3); 
        desiredPose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
        % Update initial guess for IK
        if t > 1
            initialGuess = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [jointAngles, ~] = ik(endEffector, desiredPose, weights, initialGuess);
        
        % Apply damping to the joint movements (if t > 1)
        if t > 1
            dampedJointAngles = dampingFactor * jointAngles(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = dampedJointAngles - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            jointAngles(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store IK results
        j1_com(t, 1) = jointAngles(1);
        j2_com(t, 1) = jointAngles(2);
        j3_com(t, 1) = jointAngles(3);
        j4_com(t, 1) = jointAngles(4);
        j5_com(t, 1) = jointAngles(5);
        o6 = jointAngles(6); o7 = jointAngles(7); o8 = jointAngles(8); o9 = jointAngles(9); o10 = jointAngles(10); o11 = jointAngles(11);
        
        % Store desired position for comparison
        desiredPositions = [desiredPositions; datapoint_command];
        
        % Forward Kinematics using joint angles from IK
        tform = getTransform(robot, jointAngles, endEffector);
        actualPosition = tform2trvec(tform);
        
        % Store actual FK position for comparison
        actualPositions = [actualPositions; actualPosition];

        % --------- Response Dataset (chosen_dataset) ---------
        datapoint_response = [data(4, t), data(5, t), data(6, t)];
        targetOrientation = eye(3); 
        desiredPose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);
        
        % Update initial guess for IK
        if t > 1
            initialGuess = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for response
        [jointAngles, ~] = ik(endEffector, desiredPose, weights, initialGuess);
        
        % Apply damping to the joint movements (if t > 1)
        if t > 1
            dampedJointAngles = dampingFactor * jointAngles(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = dampedJointAngles - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            jointAngles(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store IK results for response
        j1_com(t, 1) = jointAngles(1);
        j2_com(t, 1) = jointAngles(2);
        j3_com(t, 1) = jointAngles(3);
        j4_com(t, 1) = jointAngles(4);
        j5_com(t, 1) = jointAngles(5);
        o6 = jointAngles(6); o7 = jointAngles(7); o8 = jointAngles(8); o9 = jointAngles(9); o10 = jointAngles(10); o11 = jointAngles(11);
        
        % Store desired position for comparison
        desiredPositions_resp = [desiredPositions_resp; datapoint_response];
        
        % Forward Kinematics using joint angles from IK
        tform = getTransform(robot, jointAngles, endEffector);
        actualPosition_resp = tform2trvec(tform);
        
        % Store actual FK position for comparison
        actualPositions_resp = [actualPositions_resp; actualPosition_resp];
    end
    
    % Combine the FK results for command and response
    output_cell_array{i} = [actualPositions, actualPositions_resp];
end
