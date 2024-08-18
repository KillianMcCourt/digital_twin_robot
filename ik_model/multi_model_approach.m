
correctDTWpredcount = 0;
totalDTWpredcount = 0;
realistic_trajs_number = 100
nb_points_traj = 1000
zero_amount = 0
start_index = 201; % Starting from the 201st point
end_index = 800;   % Ending at the 800th point
centering = true;
displays = true;
use_case ="5D";
close all
numClasses = 9;
stationary_error = 0.628;
real_dataset = [];
real_cell_dataset = {};
simulated_dataset = [];
baseDir = '/home/therandomheretek/Desktop/shadow_fork/dtr_robot_digital_shaddow/pc_side/catkin_ws/src/cm_listener/data_repository';
confusionMatrixReal = zeros(numClasses);
confusionMatrixSimulated = zeros(numClasses);
% Get a list of all subfolders within the base directory
subFolders = dir(baseDir);
subFolders = subFolders([subFolders.isdir]); % Keep only directories
subFolders = subFolders(~ismember({subFolders.name}, {'.', '..'})); % Remove '.' and '..'
mse_predictions = [];
motorwise_mse_predictions = [];
% Sort the folder names in ascending order
[~, sortIdx] = sort({subFolders.name});
subFolders = subFolders(sortIdx);
true_labels = [];
predicted_labels = [];

% Initialize confusion matrix

confusionMatrix = zeros(numClasses);
stationary_error_timestap = 100;

%%
[realistic_trajs_set,realistic_trajs_representative_point_set]= createRandomPickupList(realistic_trajs_number,nb_points_traj, zero_amount);

% Loop through each subfolder
for k = 1:length(realistic_trajs_set)
    shape=realistic_trajs_set{k};  
            j1=shape{1};
            j2=shape{2};
            j3=shape{3};
            j4=shape{4}; 
            j5=shape{5};
   
    

  


        
        
        len_time_series = 1000; % because most commands don't contain point-by-point commands, this has to be specified by the user
        
      
      

        
        
        
        sample_time=10/len_time_series;
        max_speed =2.7;  %empirical value 2.2642; 
        %degrees per point, i.e. here 2.7 deg / 10ms
        %in theory could be 2.7*(timestamp(end)-timestamp(1))/10 to account for
        %other simulation time to point ratio
        true_pred = 0;
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
                j1 =  180+ j1;
                j2 =  -j2;
                j3 =  j3;
                j4 =  -j4;
                j5 =  j5;
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
    results = zeros(numClasses,1);
    results_motorwise = zeros(numClasses,1);
    mse_values = zeros(numClasses, 3);

     for ki=1:numClasses   
         disp("new ki = ")
         disp(ki)
         disp("--")

     switch ki
        case 1
            error1 = m1;
            assignin('base', 'error1', error1)

        case 2
            joint1_ts.Data = process_points(joint1_ts.Data);
        
        case 3
            joint2_ts.Data = process_points(joint2_ts.Data);
            
        case 4
            joint3_ts.Data = process_points(joint3_ts.Data);
          
        case 5
            joint4_ts.Data = process_points(joint4_ts.Data);
         
                % case 6
                %     joint1_ts.Data = extend_trajectory(joint1_ts.Data, rand());
                % 
                % case 7
                %     joint2_ts.Data = extend_trajectory(joint2_ts.Data, rand());
                % 
                % case 8
                %     joint3_ts.Data = extend_trajectory(joint3_ts.Data, rand());
                % 
                % case 9
                %     joint4_ts.Data = extend_trajectory(joint4_ts.Data, rand());
                
              case 6
                    joint1_ts.Data = process_points_stationary_error(joint1_ts.Data, stationary_error, stationary_error_timestap);
                    
                case 7
                    joint2_ts.Data = process_points_stationary_error(joint2_ts.Data, -stationary_error, stationary_error_timestap);
                  
                case 8
                     joint3_ts.Data = process_points_stationary_error(joint3_ts.Data, stationary_error, stationary_error_timestap);
                    
                case 9
                    joint4_ts.Data = process_points_stationary_error(joint4_ts.Data, -stationary_error, stationary_error_timestap);
                
     end












%%

% Assign modified time series back to workspace variables (if needed)


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
pause(5);
joint1_ts = joint_ts{1};
joint2_ts = joint_ts{2};
joint3_ts = joint_ts{3};
joint4_ts = joint_ts{4};
joint5_ts = joint_ts{5};

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
        
        

        
        %% Results
        

            [comparison_matrix, x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o,j5o,len_time_series); 
            [comparison_matrix_target, x_target, y_target, z_target] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series); 
           
  
            simulated_datapoint = [comparison_matrix_target, comparison_matrix];
            net = load('trained_AI_model.mat');
            net = net.net;
       
            prediction_simulated= net.predict(simulated_datapoint');
            simulated_dataset = [simulated_dataset, simulated_datapoint]

    

% Display the indexes


end
  
  

    
  

  
   
end
save('mm_dataset','real_dataset')




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
index = 1;
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
    for i = 2:length(x_values) - 1
    if x_values(i) == 0
        x_values(i) = (x_values(i - 1) + x_values(i + 1)) / 2;
    end
    end
    interpolated_x = linspace(min(x_values), max(x_values), len_time_series);
    interpolated_y = interp1(x_values, y_values, interpolated_x);
    
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];

    % Interpolate the response to len_time_series

    %response_positions = pchip(df.time_since_start, df.(motor), interpolated_x)
    %interpolated_x); -seems to induce strong oscillations in x-response

    response_positions = interp1(df.time_since_start, df.(motor), interpolated_x);
    for i = 2:length(response_positions)
    if isnan(response_positions(i))
        response_positions(i) = response_positions(i-1);
    end
    end

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


    %pointsList = [ones(200,1);zeros(800, 1)];
    pointsList = [zeros(1000, 1)];


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
    command_time = df_cmd.timestamp;
    response = df.(motor);
    response_time = df.timestamp;

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
function [index_real, index_simulated] = getPredictionIndexes(real_datapoint, simulated_datapoint, net)
    % Predict the values
    prediction_real = net.predict(real_datapoint');
    prediction_simulated = net.predict(simulated_datapoint');

    % Find the indexes of the highest values
    [~, index_real] = max(prediction_real);
    [~, index_simulated] = max(prediction_simulated);
end




function plotRealDataPoints(real_datapoint)
    % Ensure the input is a 1000x6 matrix
    if size(real_datapoint, 1) ~= 1000 || size(real_datapoint, 2) ~= 6
        error('Input matrix must be 1000x6 in size.');
    end

    % Create a figure
    figure;

    % Plot 1st and 4th columns
    subplot(3, 1, 1); % Create a subplot with 3 rows, 1 column, position 1
    plot(real_datapoint(:, 1), 'r'); % Plot 1st column in red
    hold on;
    plot(real_datapoint(:, 4), 'b'); % Plot 4th column in blue
    title('Plot of 1st and 4th Columns');
    legend('1st Column', '4th Column');
    xlabel('Index');
    ylabel('Value');
    hold off;

    % Plot 2nd and 5th columns
    subplot(3, 1, 2); % Create a subplot with 3 rows, 1 column, position 2
    plot(real_datapoint(:, 2), 'r'); % Plot 2nd column in green
    hold on;
    plot(real_datapoint(:, 5), 'b'); % Plot 5th column in magenta
    title('Plot of 2nd and 5th Columns');
    legend('2nd Column', '5th Column');
    xlabel('Index');
    ylabel('Value');
    hold off;

    % Plot 3rd and 6th columns
    subplot(3, 1, 3); % Create a subplot with 3 rows, 1 column, position 3
    plot(real_datapoint(:, 3), 'r'); % Plot 3rd column in cyan
    hold on;
    plot(real_datapoint(:, 6), 'b'); % Plot 6th column in black
    title('Plot of 3rd and 6th Columns');
    legend('3rd Column', '6th Column');
    xlabel('Index');
    ylabel('Value');
    hold off;


end



function match = displaySummaryWithRealClass(currentSubFolder, results, numClasses)
    % Extract the real class from the filename
    [real_class, minOverMeanRatio] = extract_failure_mode(currentSubFolder, results, numClasses);
    real_class = real_class+ 1 %compensating for 0 indexing
    % Calculate summary data
    summary_data = [(1:numClasses)', results.*1];
    
    % Find the index of the minimum MSE value
    [~, minIndex] = min(summary_data(:, 2));
    
    % Create a summary figure
    figure('Name', 'Summary Figure', 'NumberTitle', 'off');
    
    % Display the summary information
    summaryText = sprintf('Real Class: %d\nLowest MSE Class: %d\nMatch: %s \nPred quality %d', ...
                           real_class, minIndex, ...
                          logical(real_class == minIndex),...
                          minOverMeanRatio);
    match =(real_class == minIndex)
    % Display text in the figure
    annotation('textbox', [0.1, 0.5, 0.8, 0.3], 'String', summaryText, 'FontSize', 12, ...
               'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'EdgeColor', 'none');
end


function [real_class, minOverNextRatio] = extract_failure_mode(currentSubFolder, results, numClasses)
    % Extract the filename from the full path
    [~, fileName, ~] = fileparts(currentSubFolder);

    % Initialize the real_class variable
    real_class = NaN;  % Use NaN as default in case no match is found

    % Define regular expression patterns for different failure modes
    patternNoFail = '^\d+_nofail_traj_\d+$';
    patternStuck = '^\d+_stuck_motor_(\d+)_traj_\d+$';
    patternSteady = '^\d+_steady_motor_(\d+)_traj_\d+$';

    % Check for nofail pattern
    if ~isempty(regexp(fileName, patternNoFail, 'once'))
        real_class = 0; % nofail = failure mode 0

    % Check for stuck motor pattern
    elseif ~isempty(regexp(fileName, patternStuck, 'once'))
        % Extract the motor number from the filename
        tokens = regexp(fileName, patternStuck, 'tokens');
        motorNumber = str2double(tokens{1}{1});
        real_class = motorNumber; % stuck motor X = failure mode X

    % Check for steady motor pattern
    elseif ~isempty(regexp(fileName, patternSteady, 'once'))
        % Extract the motor number from the filename
        tokens = regexp(fileName, patternSteady, 'tokens');
        motorNumber = str2double(tokens{1}{1});
        real_class = motorNumber + 4; % steady motor X = failure mode X + 4
    end

    % Compute the minimum and next smallest values of the results
    sortedResults = sort(results);
    minMSE = sortedResults(1); % Smallest MSE
    nextMinMSE = sortedResults(2); % Second smallest MSE

    % Compute the ratio of the minimum to the next smallest
    minOverNextRatio = minMSE / nextMinMSE;

    % Display the real class and ratio for debugging
    fprintf('File: %s, Real Class: %d, Min/Next Min Ratio: %.2f\n', fileName, real_class, minOverNextRatio);
end


function [trajectories,csv_file_equivalent] = createRandomPickupList(number_of_pickup_trajctories,len_time_series, zero_amount)
%_____
%Creates a structure containing a wanted number of  random trajectories that mimic
%a realistic pickup mouvement of a wanted length ( 5 commands, 1 for each
%motor)
%   Entrées:
%   --------
%
%   number_of_pickup_trajectories: number of trajectories that are given
%   back
%   len_time_series: number of points intrajectories that are given back
%   zero_amount:amount of trajectories generated without any activated
%   mouvements
%
%   Sorties:
%   --------
%
%   trajectories: array containing the generated trajectory commands of
%   wanted len
%   csv_file_equivalent: the segment by segment equivalent that the real
%   robot interacts with
%_____

%Setting  default value to average_smallest_motive_lenght

%Interpolation set creation

    % Initialize a cell array to store the trajectories
    trajectories = cell(1, number_of_pickup_trajctories);
    csv_file_equivalent = cell(1, number_of_pickup_trajctories);
    

    % Initialize counter
    generated_trajectories = 0;

    % Keep generating trajectories until the desired number is reached
 while generated_trajectories < number_of_pickup_trajctories
        
        % Increment the counter
        generated_trajectories = generated_trajectories + 1;
        
        trajectory = cell(1, 5);
        triplets_cell = cell(1, 5);
        %Iterating over each of the 5 motors to create one trajectory
        allowed_amplitude = {50, 50, 50, 50, 50};
        danger_zone = false;
        for i = 1:5
            if danger_zone == true
                allowed_amplitude = {50, 50, 50, 50, 50};
            end
            [motor_command,triplets] = realisticsinglemotorcommand(len_time_series,zero_amount, allowed_amplitude{i});
            if i ==1
                
                if -50<triplets(1,1)<50
                    danger_zone = true;
                end
            end
            trajectory{i} = motor_command;
            triplets_cell{i} = triplets;
        end

        trajectories{generated_trajectories} = trajectory;
        csv_file_equivalent{generated_trajectories} = triplets_cell;
       
    end
end


%function 17
function [motor_command, triplets] = realisticsinglemotorcommand(len_time_series, zero_amount, allowed_amplitude)
%_______________
%Returns the keypoints that are representative of a pickup movement and the command for the trajectory, these
%can be used to make the real robot perform the trajectory. It is
%considered that the robot is initially in a random position, will have to
%move to a position to pick up an object and then move that object to a
%final position.
%
%   n: case that is executed 
%_______________

% Point generation range (motor command amplitude)

speed_cap = 1;

% Random number of motives on the command of each motor
% Note to self: maybe favorise appearance of 0 more often with better mechanism?

% Initialising the points at 0
motor_command = zeros(len_time_series, 1);

% Choice of motor use or not
percentage_zero_amount = zero_amount * 100;
toggle_value = generateRandomNumbers(1, 100, 1);

% Application of the toggle
if toggle_value >= percentage_zero_amount
    
    number_of_motives_in_traj = 5;
    triplets = zeros(number_of_motives_in_traj, 3);

    % Needs testing
    remaining_points = len_time_series;
    current_index = 0;
    old_point = 0;
    for i = 1:number_of_motives_in_traj
        % The two durations (one time top arrive at the point associated with the
        % motive and one for the plateau after attaining this point)
        
        % Inside motif arrival to plateau len rapport
        arrival_to_plateau_proportion = randi([2, 9]) / 10;
    
        % Motive length
        motive_len = 0.2*len_time_series;
    
        % Arrival to point length
        arrival_to_point_length = round(motive_len * arrival_to_plateau_proportion);
        triplets(i, 2) = arrival_to_point_length;
        if i ~= 1
            old_point = motor_command(current_index, 1);
        end 
        
        % Plateau length
        plateau_length = motive_len - arrival_to_point_length;
        triplets(i, 3) = plateau_length;

        % Randomly created next point
        
        if rand() < 0.1
            point = rand() * allowed_amplitude;
        else
            
            point =  - rand() * allowed_amplitude;
        end
        
        % Enforce boundary conditions
        min_point =  old_point - speed_cap * arrival_to_point_length;
        max_point = old_point + speed_cap * arrival_to_point_length;
        
        % Adjust the point to respect speed constraint
        if point < min_point
            point = min_point;
        elseif point > max_point
            point = max_point;
        end
        
        triplets(i, 1) = point;

        % Motor command updates
        motor_command(current_index + 1:current_index + arrival_to_point_length, 1) = linspace(old_point, point, arrival_to_point_length);
        motor_command(current_index + arrival_to_point_length + 1:current_index + motive_len, 1) = point;

        % Calculation of remaining points to be used 
        remaining_points = remaining_points - motive_len;
        current_index = len_time_series - remaining_points;
    end
    else
        triplets = [0, 0, 0];
        motor_command(1) = 0.0001;
        motor_command(1) = 0.0002;
    end

end   
function randomIntegers = generateRandomNumbers(a, b, p)
    % Generate p random integers in the interval [a, b]
    randomIntegers = randi([round(a), round(b)], 1, p);
end
