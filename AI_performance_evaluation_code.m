%% Blurb for readme

%This function uses real provided data (command and response trajectory) in
%order to 1 - adapt for use with our AI models (real_datapoint output) and 
% 2- generate a simulated response to the real command, allowing direct
% comparison to the real datapoint in order to access digital twin
% accuracy.
%This function can also be switched to a "3D" mode, in which 

%%
% these are placeholder points, with no impact on the simulation unless
% use_case is set to "3D" trajectry control, rather than "5D" motor
% control
use_case ="5D";
close all
numClasses = 13;
stationary_error = 0.3;
real_dataset = {};
simulated_dataset = {};
baseDir = '/home/therandomheretek/Desktop/shadow_fork/dtr_robot_digital_shaddow/pc_side/catkin_ws/src/cm_listener/data_repository';
confusionMatrixReal = zeros(numClasses);
confusionMatrixSimulated = zeros(numClasses);
% Get a list of all subfolders within the base directory
subFolders = dir(baseDir);
subFolders = subFolders([subFolders.isdir]); % Keep only directories
subFolders = subFolders(~ismember({subFolders.name}, {'.', '..'})); % Remove '.' and '..'

% Sort the folder names in ascending order
[~, sortIdx] = sort({subFolders.name});
subFolders = subFolders(sortIdx);

% Initialize confusion matrix
numClasses = 13;
confusionMatrix = zeros(numClasses);
stationary_error_timestap = 100;

% Loop through each subfolder
for k = 1:length(subFolders)
    % Get the current subfolder name
    currentSubFolder = fullfile(baseDir, subFolders(k).name);
    
    % Check if the subfolder is empty
    if isempty(dir(fullfile(currentSubFolder, '*.csv')))
        % If the subfolder does not contain any CSV files, skip it
        continue;
    end
    
    % Construct the file paths for the CSV files in the current subfolder
    positionFile = fullfile(currentSubFolder, 'trajectory_monitoring_position.csv');
    cmdFile = fullfile(currentSubFolder, 'trajectory_monitoring_cmd.csv');
    cmdDurationFile = fullfile(currentSubFolder, 'trajectory_monitoring_cmd_duration.csv');
    
    % Check if the files exist before reading them
    if isfile(positionFile) && isfile(cmdFile) && isfile(cmdDurationFile)
        % Read the tables from the CSV files
        df = readtable(positionFile);
        df_cmd = readtable(cmdFile);
        df_cmd_duration = readtable(cmdDurationFile);
        
        % Add your processing code here     


        %% preparing data for full trajectory test
        
        %specify source files for the trajectory
        
        
        len_time_series = 1000; % because most commands don't contain point-by-point commands, this has to be specified by the user
        
        
        %the following variables represent three provided .csv tables : the real
        %response df, the real command points df_cmd, and the associated durations
        %for each as df_cmd_duration.
        
        
        
        % Calculate time since start for both dataframes
        df.time_since_start = df.timestamp - df.timestamp(1);
        %disp(df.time_since_start)
        df_cmd.time_since_start = max(0,df_cmd.timestamp - df.timestamp(1));
        
        
        % Convert position values to the axis system used in our simulations
        df_deg = df{:, 2:end} / 1000 * 240;
        df_cmd_deg = df_cmd{:, 2:end} / 1000 * 240;
           
        
        %for this section xyz_targets is simply a .csv containing positions along
        %each axis for the target positions. The code will perform IK-FK operations
        %to simulate. Comparison with real response not yet supported.
        
        
        
        
        %% Commands 
        %this section calls generate_positions to turn the provided points into
        %exploitable time series. See function definitions for details on the
        %operations.
        
        [com_motor_6, mov_6] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(6)],len_time_series);
        [com_motor_5, mov_5] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(5)],len_time_series);
        [com_motor_4, mov_4] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(4)],len_time_series);
        [com_motor_3, mov_3] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(3)],len_time_series);
        [com_motor_2, mov_2] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(2)],len_time_series);

        display_curves(df, df_cmd, ['motor_' num2str(6)])
        display_curves(df, df_cmd, ['motor_' num2str(5)])
        display_curves(df, df_cmd, ['motor_' num2str(4)])
        display_curves(df, df_cmd, ['motor_' num2str(3)])
        display_curves(df, df_cmd, ['motor_' num2str(2)])
        % 
      disp("com graphing")
      plot_comparison(com_motor_6, mov_6, "COMGRAPHING");
        
        % assuming consistent timestamps, we drop the time reference. 
        com_motor_6 = com_motor_6(:,2);
        com_motor_5 = com_motor_5(:,2);
        com_motor_4 = com_motor_4(:,2);
        com_motor_3 = com_motor_3(:,2);
        com_motor_2 = com_motor_2(:,2);
        
        
        
        sample_time=10/len_time_series;
        max_speed =2.7;  %empirical value 2.2642; 
        %degrees per point, i.e. here 2.7 deg / 10ms
        %in theory could be 2.7*(timestamp(end)-timestamp(1))/10 to account for
        %other simulation time to point ratio
        
        %Prepwork
        model_name = 'main3_armpi_fpv';
        load_system(model_name);
        joint1_damping = 0;
        joint2_damping = 0;
        damp_pince = 0;
        mdl = "robot_model";
        load_system(mdl)
        ik = simscape.multibody.KinematicsSolver(mdl);
        base = "robot_model/World/W";
        follower = "robot_model/gripper_base/F";
        addFrameVariables(ik,"gripper_base","translation",base,follower);
        addFrameVariables(ik,"gripper_base","rotation",base,follower);
        targetIDs = ["gripper_base.Translation.x";"gripper_base.Translation.y";...
            "gripper_base.Translation.z"];
        addTargetVariables(ik,targetIDs);
        outputIDs =["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"];
        addOutputVariables(ik,outputIDs);
        guessesIDs = ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"];
        guesses = [3,3,3,3,3];
        addInitialGuessVariables(ik,guessesIDs);
        
        %simul length: len_time_series/100= legnth of simulation in seconds
        
        j1 = zeros(len_time_series,1);
        j2 = zeros(len_time_series,1);
        j3 = zeros(len_time_series,1);
        j4 = zeros(len_time_series,1);
        j5 = zeros(len_time_series,1);
        spline = zeros(len_time_series,3);
        targets = zeros(len_time_series,3);
        
        
        m0=[transpose(1:len_time_series), zeros(len_time_series, 1)];
        m1=[transpose(1:len_time_series), ones(len_time_series, 1)];
        
        if use_case =="5D" 
        % in order to utilise the same base structure, I found it simpler to
        % generate a placeholder 3D traj rather than handle undefined x, y, z later
        % on. This has no impact on the the actual simulation.
        vector = 0.01*rand(len_time_series, 1);
        x_placeholder = vector;
        y_placeholder = vector;
        z_placeholder = vector;
        x_placeholder=x_placeholder+0.11;
        y_placeholder=y_placeholder+0.11;
        z_placeholder=z_placeholder+0.01;
        datapoints =[x_placeholder, y_placeholder,z_placeholder];
        
        elseif use_case=="3D"
        datapoints = xyz_targets;
        end
        
        
        %% start of the simulation 
        
         for t = 1:len_time_series
                datapoint =datapoints(t,:);
                spline(t,:)  = datapoint;
                targets(t,:) = datapoint;
        
                
                
                if t>1 
                    guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint,guesses);
        
                if use_case =="5D"
                j1(t,1) =  180+ (com_motor_6(t)-500)*0.24;
                j2(t,1) =  -(com_motor_5(t)-500)*0.24;
                j3(t,1) =  (com_motor_4(t)-500)*0.24;
                j4(t,1) =  -(com_motor_3(t)-500)*0.24;
                j5(t,1) =  (com_motor_2(t)-500)*0.24;
                elseif use_case =="3D"
                j1(t,1) = outputVec(1);
                j2(t,1) = outputVec(2);
                j3(t,1) = outputVec(3);
                j4(t,1) = outputVec(4);
                j5(t,1) = outputVec(5);
                end
        
                
        
         end
        
          
          j1 = process_points_capped_speed(j1, max_speed);
          j2 = process_points_capped_speed(j2, max_speed);
          j3 = process_points_capped_speed(j3, max_speed);
          j4 = process_points_capped_speed(j4, max_speed);
          j5 = process_points_capped_speed(j5, max_speed);
        
        end_time_value_in_seconds= (len_time_series-1)*0.01;
        
        motor_command_matrix = [j1,j2,j3,j4,j5];
        writematrix(motor_command_matrix,'motor_command_matrix.csv')
        
        joint1_ts = timeseries(j1/180*pi,0:0.01:end_time_value_in_seconds);
        joint2_ts = timeseries(j2/180*pi,0:0.01:end_time_value_in_seconds);
        joint3_ts = timeseries(j3/180*pi,0:0.01:end_time_value_in_seconds);
        joint4_ts = timeseries(j4/180*pi,0:0.01:end_time_value_in_seconds);
        joint5_ts = timeseries(j5/180*pi,0:0.01:end_time_value_in_seconds);
        

% Define time series for each joint
joint_ts = { ...
    timeseries(j1/180*pi, 0:0.01:end_time_value_in_seconds), ...
    timeseries(j2/180*pi, 0:0.01:end_time_value_in_seconds), ...
    timeseries(j3/180*pi, 0:0.01:end_time_value_in_seconds), ...
    timeseries(j4/180*pi, 0:0.01:end_time_value_in_seconds), ...
    timeseries(j5/180*pi, 0:0.01:end_time_value_in_seconds) ...
};

% Number of points
n_points = length(joint_ts{1}.Data);
segments = 5;
points_per_segment = n_points / segments;
selected_segments = randperm(segments, 3);

%%


% switch mod(k, 13) +1
%     case 1
%         % No failure
%         error1 = m0; error2 = m0; error3 = m0; error4 = m0; error5 = m0; error6 = m0;
% 
%     case {7,8,9,10}
%         % Block motor motion by keeping position constant
%         motor_index = mod(k - 1, 5) + 1;
%         joint_ts{motor_index}.Data = joint_ts{motor_index}.Data(1) * ones(n_points, 1);
% 
%     case {2,3,4,5,6}
%         % Motor stutter: Keep position constant during selected segments
%         motor_index = mod(k - 7, 5) + 1;
%         joint_data = joint_ts{motor_index}.Data;
%         for seg = selected_segments
%             start_idx = (seg - 1) * points_per_segment + 1;
%             end_idx = start_idx + points_per_segment - 1;
%             % Hold position constant during segment
%             joint_data(start_idx:end_idx) = joint_data(start_idx);
%         end
%         joint_ts{motor_index}.Data = joint_data;
% 
%     case {12, 13, 14, 15, 16}
%         % Motor steady state error: Add constant error during selected segments
%         motor_index = mod(k - 12, 5) + 1;
%         steady_state_error = 30 * 0.24 * pi / 180; % Convert degrees to radians
%         joint_data = joint_ts{motor_index}.Data;
%         for seg = selected_segments
%             start_idx = (seg - 1) * points_per_segment + 1;
%             end_idx = start_idx + points_per_segment - 1;
%             % Add steady state error during segment
%             joint_data(start_idx:end_idx) = joint_data(start_idx:end_idx) + steady_state_error;
%         end
%         joint_ts{motor_index}.Data = joint_data;
% end

%%
     switch mod(k, 13) 
        case 0
            error1 = m1;
            assignin('base', 'error1', error1)

        case 1
            joint1_ts.Data = process_points(joint1_ts.Data);
         
        case 2
            joint2_ts.Data = process_points(joint2_ts.Data);
          
        case 3
            joint3_ts.Data = process_points(joint3_ts.Data);
        
        case 4
            joint4_ts.Data = process_points(joint4_ts.Data);
         
        case 5
            joint5_ts.Data = process_points(joint5_ts.Data);
         
    
                case 6
                    joint1_ts.Data = extend_trajectory(joint1_ts.Data, rand());
                
                case 7
                    joint2_ts.Data = extend_trajectory(joint2_ts.Data, rand());
                 
                case 8
                    joint3_ts.Data = extend_trajectory(joint3_ts.Data, rand());
                 
                case 9
                    joint4_ts.Data = extend_trajectory(joint4_ts.Data, rand());
                
              case 10
                    joint1_ts.Data = process_points_stationary_error(joint1_ts.Data, stationary_error, stationary_error_timestap);
                    
                case 11
                    joint2_ts.Data = process_points_stationary_error(joint2_ts.Data, stationary_error, stationary_error_timestap);
                  
                case 12
                     joint3_ts.Data = process_points_stationary_error(joint3_ts.Data, stationary_error, stationary_error_timestap);
                    
                case 13
                    joint4_ts.Data = process_points_stationary_error(joint4_ts.Data, stationary_error, stationary_error_timestap);
                
            end












%%

% Assign modified time series back to workspace variables (if needed)
% joint1_ts = joint_ts{1};
% joint2_ts = joint_ts{2};
% joint3_ts = joint_ts{3};
% joint4_ts = joint_ts{4};
% joint5_ts = joint_ts{5};

% Add trajectories to the model
disp("----------------")
disp("----------------")
w = warning('off', 'all');
error1 = m1;
error2 = m1;
error3 = m1; 
error4 = m1;
error5 = m1; 
error6 = m1;
simOut = sim(model_name);


            warning(w);
            disp("----------------")
            disp("----------------")
            
        
            j1o = simOut.j1.Data;
            j2o = simOut.j2.Data;
            j3o = simOut.j3.Data;
            j4o = simOut.j4.Data;
            j5o = simOut.j5.Data;
            j1o = j1o*180/pi;
            j2o = j2o*180/pi;
            j3o = j3o*180/pi;
            j4o = j4o*180/pi;
            j5o = j5o*180/pi;
        
        
            j1_real =  (mov_6-500)*0.24+180;
        j2_real = -(mov_5-500)*0.24;
        j3_real = (mov_4-500)*0.24;
        j4_real = -(mov_3-500)*0.24;
        j5_real = (mov_2 -500)*0.24;
        
        %% Results
        
        %the following matrices serve to represent in a 3D plane the outputs. This
        %allows comparisons of the actual trajectories, which may be required in
        %lieu of comparing motor angular positions.
            %disp(j1o)
            %disp(j1_real)
            [comparison_matrix, x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o,j5o,len_time_series); 
            [comparison_matrix_target, x_target, y_target, z_target] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series); 
            [comparison_matrix_real, x_real, y_real, z_real] = ForwardKinematic(j1_real, j2_real, j3_real, j4_real, j5_real,len_time_series); 
           
            disp("J graphing")
            
            plot_comparison(x_target, x_real, "XGRAPH")
            plot_comparison(x_target, x, "XGRAPH")

            real_datapoint = [comparison_matrix_target, comparison_matrix_real];
            %simulated_datapoint = [comparison_matrix_target, comparison_matrix];
            simulated_datapoint = [comparison_matrix_target, comparison_matrix_target];
            % Check if the cell arrays exist, if not, initialize them
        if ~exist('real_dataset', 'var')
            real_dataset = {};
        end
        if ~exist('simulated_dataset', 'var')
            simulated_dataset = {};
        end
        
        % Append the new datapoints to the cell arrays
        real_dataset{end+1} = real_datapoint;
        simulated_dataset{end+1} = simulated_datapoint;
        
        net = load('recent_lstm_13_classes.mat');
        net = net.net;
        %% preparing data for full trajectory test
        % Assume 'net' is your neural network and 'real_datapoint' is prepared
        % from your data for prediction
          %% preparing data for full trajectory test
        % Assume 'net' is your neural network and 'real_datapoint' is prepared
        % from your data for prediction
        prediction_real = net.predict(real_datapoint'); % Assuming this is a row vector of scores
        [~, predictedClassReal] = max(prediction_real); % Find the index of the maximum score
        trueClass = k; % True class based on folder order (1-13)
        disp(prediction_real)
        
        % Update the confusion matrix for real data
        if trueClass >= 1 && trueClass <= numClasses 
            confusionMatrixReal(trueClass, predictedClassReal) = confusionMatrixReal(trueClass, predictedClassReal) + 1;
            disp(confusionMatrixReal(trueClass, predictedClassReal))
        end
        
        % Prediction for simulated data
        prediction_simulated = net.predict(simulated_datapoint'); % Assuming this is a row vector of scores
        [~, predictedClassSimulated] = max(prediction_simulated); % Find the index of the maximum score
        
        % Update the confusion matrix for simulated data
        if trueClass >= 1 && trueClass <= numClasses && ~isnan(predictedClassSimulated)
            confusionMatrixSimulated(trueClass, predictedClassSimulated) = confusionMatrixSimulated(trueClass, predictedClassSimulated) + 1;
        end
        
        %% Your plotting and further processing code
        j1o = j1o / 0.24 + 500;
        j2o = j2o / 0.24 + 500;
        j3o = j3o / 0.24 + 500;
        j4o = j4o / 0.24 + 500;
        j5o = j5o / 0.24 + 500;
        
        for i = 2:6
            plot_motor_movement_3(df, df_cmd, df_cmd_duration, ['motor_' num2str(i)], eval(['j' num2str(7-i) 'o']));
        end
    
end

% Display or save the confusion matrices
figure;
subplot(1,2,1);
heatmap(confusionMatrixReal);
title('Confusion Matrix for Real Data');

subplot(1,2,2);
heatmap(confusionMatrixSimulated);
title('Confusion Matrix for Simulated Data');

% Calculate accuracy for real data
totalPredictionsReal = sum(confusionMatrixReal(:));
correctPredictionsReal = sum(diag(confusionMatrixReal));
accuracyReal = correctPredictionsReal / totalPredictionsReal;
fprintf('Accuracy for Real Data: %.2f%%\n', accuracyReal * 100);

% Calculate accuracy for simulated data
totalPredictionsSimulated = sum(confusionMatrixSimulated(:));
correctPredictionsSimulated = sum(diag(confusionMatrixSimulated));
accuracySimulated = correctPredictionsSimulated / totalPredictionsSimulated;
fprintf('Accuracy for Simulated Data: %.2f%%\n', accuracySimulated * 100);
end
function [comparison_matrix, x, y ,z] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series)
    joint1_damping = 0;
    joint2_damping = 0;
    damp_pince = 1000; 
    
    
    mdl = "robot_model";
    
    load_system(mdl)
    
    ik = simscape.multibody.KinematicsSolver(mdl);
    
    base = "robot_model/World/W";
    follower = "robot_model/gripper_base/F";
    addFrameVariables(ik,"gripper_base","translation",base,follower);
    addFrameVariables(ik,"gripper_base","rotation",base,follower);
    
    targetIDs = ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"] ;
    addTargetVariables(ik,targetIDs);
    outputIDs =["gripper_base.Translation.x";"gripper_base.Translation.y";...
        "gripper_base.Translation.z"];
    addOutputVariables(ik,outputIDs);
    
    x = zeros(len_time_series,1);
    y = zeros(len_time_series,1);
    z = zeros(len_time_series,1);
    spline = zeros(len_time_series,5);
    
    len = size(j1);
    for i = 1:len_time_series
        targets = [j1(i),j2(i),j3(i),j4(i),j5(i)];
        
        % try
         [outputVec,statusFlag] = solve(ik,targets);
        % catch 
        %     disp(ik)
        %     disp(targets)
        %     counter = counter + 1
        %     disp(counter +1 )
        % end

        x(i,1) = outputVec(1);
        y(i,1) = outputVec(2);
        z(i,1) = outputVec(3);
    
        
    end
comparison_matrix = zeros(len_time_series, 3);
comparison_matrix(:, 1) = x;
comparison_matrix(:, 2) = y;
comparison_matrix(:, 3) = z;
writematrix(comparison_matrix, "realised")

% Plot the motion in 3D space of the simulated matrix as an example. Code
% can be trivially adapted to display the others.
% figure;
% plot3(comparison_matrix(:,1), comparison_matrix(:,2), comparison_matrix(:,3), 'g');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Comparison of Motions in 3D Space');
% legend( 'Comparison motion')
% grid on;

end

%the following function serves to dynamically evaluate a proposed
%trajectory, and determine whether it requires speeds higher than what is possible.
% If so, it creates a new trajectory that respects the max speed and tries
% to follow the original as closely as possible. 

function updated_j1 = process_points_capped_speed(j1, max_speed)

%sample values : cap = 2, time_scale  = 0.01
    for i = 2:numel(j1)
          
        if (j1(i)-j1(i-1)) > max_speed
            % Set j1(i) to j1(i-1)
            
            j1(i) = j1(i-1)+max_speed;
        elseif (j1(i)-j1(i-1)) < - max_speed

            j1(i) = j1(i-1)-max_speed;
            
        end
    end

    % Return the updated j1 list
    updated_j1 = j1;
end

function [reference_positions, response_positions] = generate_positions(df, df_cmd, df_cmd_duration, motor,len_time_series)

    command = df_cmd.(motor);
    disp("original command length")
    disp(size(command))
    

    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

index = find(df.time_since_start < start_time(1), 1, 'last');

% Check if the index is empty and assign the starting position accordingly
if isempty(index)
    starting_position = df{1, motor};
    %disp('empty index')
else
    starting_position = df{index, motor};
end

%This part finds the index of the last recorded time that is less than the start time of the first command.
%If such an index doesn't exist (i.e., isempty(index)), it uses the first recorded position of the motor.
%Otherwise, it uses the position of the motor at the found index.
    points = [start_time(1), starting_position];
    %disp(size(points))
    points = [points; start_time(1)+duration(1)/len_time_series, command(1)];
    for idx = 2:length(command)
        points = [points; start_time(idx), command(idx-1)];  %this places the target point
        points = [points; start_time(idx)+duration(idx)/len_time_series, command(idx)]; %this creates an articial target representing the end of the plateau
    end
    disp("points length")
    disp(size(points))
    %It initializes the points array with the first start time and the starting position.
    %For each command, it adds two points:
    %One at the start time of the command with the previous command value.
    %Another after the duration of the command divided by len_time_series with the current command value.
    %Finally, it adds a point at the end of the recorded time with the last command value.


    points = [points; df.time_since_start(end), command(end)];

    % Interpolate linearly along the given motor's positions to get len_time_series points for the reference curve
    %once time seires has been obtained, use it as a time reference for a
    %new interpolation 
    x_values = points(:, 1);
    disp("len of x values")
    disp(size(x_values))
    y_values = points(:, 2);
    interpolated_x = linspace(min(x_values), max(x_values), len_time_series);
    interpolated_y = interp1(x_values, y_values, interpolated_x);
    
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];

    % Interpolate the response to len_time_series

    response_positions = interp1(df.time_since_start, df.(motor), interpolated_x);

end



function plot_motor_movement_3(df, df_cmd, df_cmd_duration, motor, jo)
    % figure;
    % plot(df.time_since_start, df.(motor), 'LineWidth', 1.5, 'DisplayName', 'response');

    command = df_cmd.(motor);
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

    % Find the index where df.time_since_start is less than start_time(1)
index = find(df.time_since_start < start_time(1), 1, 'last');

% Check if the index is empty and assign the starting position accordingly
if isempty(index)
    starting_position = df{1, motor};
    %disp('empty index')
else
    starting_position = df{index, motor};
end
    points = [start_time(1), starting_position];
    points = [points; start_time(1)+duration(1)/1000, command(1)];
    for idx = 2:length(command)
        points = [points; start_time(idx), command(idx-1)];
        points = [points; start_time(idx)+duration(idx)/1000, command(idx)];
    end
    points = [points; df.time_since_start(end), command(end)];

    % Interpolate to get 1000 points
    x_values = points(:, 1);
    y_values = points(:, 2);
    interpolated_x = linspace(min(x_values), max(x_values), 1000);
    interpolated_y = interp1(x_values, y_values, interpolated_x);

    % Plot the points and connect them with straight lines
    % hold on;
    % plot(points(:,1), points(:,2), '-or', 'DisplayName', 'reference');
    % plot(interpolated_x, interpolated_y, 'DisplayName', 'interpolated reference');
    % plot(interpolated_x, jo', 'DisplayName', 'jo')
    % hold off;
    % 
    % legend;
    % xlabel('time since start (seconds)');
    % ylabel('position (degrees)');
    % title([motor ' movement']);
    % grid on;
    % 
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];
end


function plot_comparison(com_motor_6, mov_6, name)
    % This function plots com_motor_6 and mov_6 on three different graphs within the same figure.
    % Inputs:
    %   com_motor_6 - A vector or matrix to be plotted in the first and third subplots
    %   mov_6 - A vector or matrix to be plotted in the second and third subplots

    % Create a figure for the plots
    figure;

    % Plot com_motor_6 on the first subplot
    subplot(3, 1, 1);
    plot(com_motor_6);
    title('com\_motor\_6');
    xlabel('Index');
    ylabel('Value');

    % Plot mov_6 on the second subplot
    subplot(3, 1, 2);
    plot(mov_6);
    title('mov\_6');
    xlabel('Index');
    ylabel('Value');

    % Plot both com_motor_6 and mov_6 on the third subplot for comparison
    subplot(3, 1, 3);
    plot(com_motor_6);
    hold on;
    plot(mov_6);
    hold off;
    title('com\_motor\_6 vs mov\_6');
    xlabel('Index');
    ylabel('Value');
    legend(string(name), string(name));

    % Adjust the layout
    sgtitle(string(name));
end


function updated_j1 = process_points_stationary_error(j1, stationary_error, stationary_error_timestamp)
    % Define the length of each block
    blockSize = 200;
    
    % Number of blocks
    numBlocks = length(j1) / blockSize;
    
    % Generate a list of blocks to be updated
    blocksToUpdate = rand(numBlocks, 1) <= 1;
    
    % Ensure that at least one block will be updated
    if ~any(blocksToUpdate)
        blocksToUpdate(randi(numBlocks)) = true;
    end
    
    % Iterate through the blocks to be updated and apply the error
    for i = 1:numBlocks
        if blocksToUpdate(i)
            start_index = (i-1)*blockSize + 1;
            end_index = i*blockSize;
            j1(start_index:end_index) = j1(start_index:end_index) + stationary_error;
        end
    end
    
    % Return the updated j1
    updated_j1 = j1;
end

function updated_j1 = process_points(j1)
    % Initialize pointsList
    pointsList = zeros(length(j1), 1);

    % Track if at least one block is set to zero
    zeroBlockExists = false;

    % Loop through each block of 200 points
    for i = 1:floor(length(j1)/200)
        start_index = (i-1)*200 + 1;
        end_index = i*200;

        % Generate a random number to decide if the block will be zeros or ones
        if rand <= 1 
            pointsList(start_index:end_index) = 0;
            zeroBlockExists = true; % Set flag to true indicating at least one block is set to zero
        else
            pointsList(start_index:end_index) = 1;
        end
    end

    % If no block is set to zero, randomly select one block and set it to zero
    if ~zeroBlockExists
        blockIndex = randi(floor(length(j1)/200));
        start_index = (blockIndex-1)*200 + 1;
        end_index = blockIndex*200;
        pointsList(start_index:end_index) = 0;
    end

    % Track the indices of the last 50 zeros
    lastZerosIndices = find(pointsList == 0, 50, 'last');

    % Check if the next set in pointsList is a set of ones
    if length(pointsList) > lastZerosIndices(end) + 200 && all(pointsList(lastZerosIndices(end) + 1:lastZerosIndices(end) + 200) == 1)
        %disp("end of stoppage - interpolating to avoid jump")
        % Interpolate from the current value of j1 to the next one
        startValue = j1(lastZerosIndices(end));
        endValue = j1(lastZerosIndices(end) + 200);
        interpolatedValues = linspace(startValue, endValue, 200);
        j1(lastZerosIndices(end) + 1:lastZerosIndices(end) + 200) = interpolatedValues;
    end

    % Iterate over pointsList to set j1(i) to j1(i-1) where necessary
    %disp(pointsList)


    pointsList = [ones(200,1);zeros(800, 1)];



    for i = 2:numel(pointsList)
        if pointsList(i) == 0
            j1(i) = j1(i-1);
        end
        
    end

    % Return the updated j1 list
    updated_j1 = j1;
end


function updated_trajectory = extend_trajectory(originalPoints, scaleFactor)
    % Initialize the updated trajectory with the original points
    updated_trajectory = originalPoints;
    
    % Define the length of each block
    blockSize = 200;
    
    % Number of blocks
    numBlocks = floor(length(originalPoints) / blockSize);
    
    % Track if at least one block has been resampled
    resampleMade = false;
    
    % Iterate through each block of 200 points
    for i = 1:numBlocks
        start_index = (i-1)*blockSize + 1;
        end_index = i*blockSize;
        
        % Generate a random number to decide if the first 100 points will be resampled
        if rand <= 1 
            % Original first 100 points
            first_half = originalPoints(start_index:start_index+99);
            second_half = originalPoints(start_index+100:start_index+199);
            
            % Extend the first 100 points
            extended_points = extend_points(first_half, scaleFactor);
            
            % Downscale the second 100 points
            compressed_points = extend_points(second_half, scaleFactor);
            
            % Combine the extended and compressed points
            resampled_points = [extended_points, compressed_points];
            
            % Ensure resampled_points length matches the original block size
            if length(resampled_points) > blockSize
                resampled_points = resampled_points(1:blockSize);
            elseif length(resampled_points) < blockSize
                resampled_points = [resampled_points, zeros(1, blockSize - length(resampled_points))];
            end
            
            % Update the trajectory with resampled points
            updated_trajectory(start_index:end_index) = resampled_points;
            
            resampleMade = true; % Set flag to true indicating at least one block has been resampled
        end
    end
    
    % If no block has been resampled, randomly select one block to resample
    if ~resampleMade
        blockIndex = randi(numBlocks);
        start_index = (blockIndex-1)*blockSize + 1;
        end_index = blockIndex*blockSize;
        
        % Original first 100 points
        first_half = originalPoints(start_index:start_index+99);
        second_half = originalPoints(start_index+100:start_index+199);
        
        % Extend the first 100 points
        extended_points = extend_points(first_half, scaleFactor);
        
        % Downscale the second 100 points
        compressed_points = extend_points(second_half, scaleFactor);
        
        % Combine the extended and compressed points
        resampled_points = [extended_points, compressed_points];
        
        % Ensure resampled_points length matches the original block size
        if length(resampled_points) > blockSize
            resampled_points = resampled_points(1:blockSize);
        elseif length(resampled_points) < blockSize
            resampled_points = [resampled_points, zeros(1, blockSize - length(resampled_points))];
        end
        
        % Update the trajectory with resampled points
        updated_trajectory(start_index:end_index) = resampled_points;
    end
end

function extended_points = extend_points(points, scaleFactor)
    % Number of original points

    num_original_points = numel(points);
    % Number of points after extending
    num_extended_points = round(num_original_points * (1 + scaleFactor));
    
   % Original number of points
    
    
    % Reshape points to ensure it is a row vector
    points = reshape(points, 1, num_original_points);
    
    % New list of points
    new_points = linspace(0, 1, num_extended_points);
    

 
    % Linear interpolation to extend the original points
    extended_points = interp1(linspace(0, 1, num_original_points), points, new_points);

end


function display_curves(df, df_cmd, motor)
    % Plot the command curve
    command = df_cmd.(motor);
    command_time = df_cmd.time_since_start;
    response = df.(motor);
    response_time = df.time_since_start;

    figure;
    hold on;
    grid on;
    
    % Plot the response curve with a line
    plot(response_time, response, 'b-', 'LineWidth', 1.5);

    % Add red crosses at each command point
    plot(command_time, command, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

    % Add green crosses at the midpoint between command points
    for i = 1:length(command) - 1
        midpoint_time = (command_time(i) + command_time(i + 1)) / 2;
        previous_value = command(i);
        plot(midpoint_time, previous_value, 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    end

    % Add titles and labels
    title(['Command and Response Curves for ', motor]);
    xlabel('Time Since Start');
    ylabel('Value');
    
    hold off;
end
