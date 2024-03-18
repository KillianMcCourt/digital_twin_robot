
len_time_series=1000;
%% preparing data for full trajectory test

df = readtable('trajectory_monitoring_position.csv');
df_cmd = readtable('trajectory_monitoring_cmd.csv');
df_cmd_duration = readtable('trajectory_monitoring_cmd_duration.csv');

% Calculate time since start for both dataframes
df.time_since_start = df.timestamp - df.timestamp(1);
df_cmd.time_since_start = df_cmd.timestamp - df.timestamp(1);

% Convert position values to degrees
df_deg = df{:, 2:end} / 1000 * 240;
df_cmd_deg = df_cmd{:, 2:end} / 1000 * 240;
   




%% Commands 

%commands from real traj
% com_motor_6 = interp1(1:numel(trajectory_monitoring_position.motor_6), trajectory_monitoring_position.motor_6, linspace(1, numel(trajectory_monitoring_position.motor_6), 1000));
% com_motor_5 = interp1(1:numel(trajectory_monitoring_position.motor_5), trajectory_monitoring_position.motor_5, linspace(1, numel(trajectory_monitoring_position.motor_5), 1000));
% com_motor_4 = interp1(1:numel(trajectory_monitoring_position.motor_4), trajectory_monitoring_position.motor_4, linspace(1, numel(trajectory_monitoring_position.motor_4), 1000));
% com_motor_3 = interp1(1:numel(trajectory_monitoring_position.motor_3), trajectory_monitoring_position.motor_3, linspace(1, numel(trajectory_monitoring_position.motor_3), 1000));
% com_motor_2 = interp1(1:numel(trajectory_monitoring_position.motor_2), trajectory_monitoring_position.motor_2, linspace(1, numel(trajectory_monitoring_position.motor_2), 1000));

%commands from targets
% com_motor_6 =  reshape(repmat(trajectory_monitoring_cmd.motor_6, 1, 1000/8).',1,[]);
% com_motor_5 =  reshape(repmat(trajectory_monitoring_cmd.motor_5, 1, 1000/8).',1,[]);
% com_motor_4 =  reshape(repmat(trajectory_monitoring_cmd.motor_4, 1, 1000/8).',1,[]);
% com_motor_3 =  reshape(repmat(trajectory_monitoring_cmd.motor_3, 1, 1000/8).',1,[]);
% com_motor_2 =  reshape(repmat(trajectory_monitoring_cmd.motor_2, 1, 1000/8).',1,[]);

 
[com_motor_6, mov_6] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(6)]);
[com_motor_5,mov_5 ]=  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(5)]);
[com_motor_4, mov_4] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(4)]);
[com_motor_3, mov_3] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(3)]);
[com_motor_2, mov_2] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(2)]);
% 
% size(com_motor_6)
com_motor_6 = com_motor_6(:,2);
com_motor_5 = com_motor_5(:,2);
com_motor_4 = com_motor_4(:,2);
com_motor_3 = com_motor_3(:,2);
com_motor_2 = com_motor_2(:,2);



sample_time=10/len_time_series;
max_speed =2.7;  %empirical value 2.2642; %degrees per point, i.e. 10ms

% delay1: real 60ms early upwards and 200ms la
% delay2 =10;
% delay3 = 20 :
% delay = 20; %delay in tens of ms between command and action in the real system
% delay5= 0;
 %bloc for motor tuning
vector = 0.01*rand(1000, 1);
x = vector;
y = vector;
z = vector;
x=x+0.11;
y=y+0.11;
z=z+0.01;









%% we need to load data from the csv here
dataTable = readtable('./ik_model/collected_data_motor_2_550.csv');
timestamp = dataTable.timestamp;
position_degree = dataTable.position_degree;
command = dataTable.command;

% Plot position_degree as a function of timestamp
figure;
plot(timestamp, position_degree, 'b-', 'LineWidth', 2);
xlabel('Timestamp');
ylabel('Position Degree');
title('Position Degree vs. Timestamp');

% Plot command as a function of timestamp
figure;
plot(timestamp, command, 'r-', 'LineWidth', 2);
xlabel('Timestamp');
ylabel('Command');
title('Command vs. Timestamp');

%%  generating the equivalent targets

j1_targets = zeros(size(command));

% Variable to keep track of whether non-nil value has been encountered
nonNilEncountered = false;

% Iterate through the command list
for i = 1:numel(command)
    % If command is non-nil and non-nil has not been encountered before
    if command(i) ~= 0 && ~nonNilEncountered
        % Set j1_targets value to 12
        j1_targets(i:end) = 12;
        % Update non-nil encountered flag
        nonNilEncountered = true;
    % If command is non-nil and non-nil has been encountered before
    elseif command(i) ~= 0 && nonNilEncountered
        % Set j1_targets value to 0
        j1_targets(i:end) = 0;
    end
end

j1_targets = interp1(linspace(0, 1, numel(j1_targets)), j1_targets, linspace(0, 1, 1000))';
position_degree_resampled = interp1(linspace(0, 1, numel(position_degree)), position_degree, linspace(0, 1, 1000))';
command_from_real = round((position_degree_resampled-120)/12) * 12;
new_targets = j1_targets-j1_targets(1);

%% 




datapoints =[x, y,z];




% x= chosencell(4,:)';
% y= chosencell(5,:)';
% z= chosencell(6,:)';
% sample_time=10/len_time_series;
% %trajectoire CS
% 
% x = trajCS.x;
% y = trajCS.y;
% z = trajCS.z;
% scaleFactorX = 3;
% scaleFactorY = 3;
% 
% % Scaling the vectors
% x_scaled = x * scaleFactorX;
% y_scaled = y * scaleFactorY;
% Plotting in 3D
%figure;
% plot3(x_scaled, y_scaled, z, 'o-');
% plot3(x, y, z, 'o-');
% grid on;
% xlabel('X-axis');
% ylabel('Y-axis');
% zlabel('Z-axis');
% title('3D Plot Example, what was done');

% % Desired number of points
% desiredPoints = len_time_series;
% 
% % Create indices for downsampling
% indices = round(linspace(1, length(x), desiredPoints));
% 
% % Downsample the vectors
% x_downsampled = x_scaled(indices);
% y_downsampled = y_scaled(indices);
% z_downsampled = z(indices)+0.05;
% 
% % Plotting downsampled in 3D
% figure;
% plot3(x_downsampled, y_downsampled, z_downsampled, 'o-');
% grid on;
% xlabel('X-axis');
% ylabel('Y-axis');
% zlabel('Z-axis');
% title('3D Plot downsampleed');
% 

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


datapoints = interp_points;

 for t = 1:len_time_series
        datapoint =datapoints(t,:);
        spline(t,:)  = datapoint;
        targets(t,:) = datapoint;

        
        
        if t>1 
            guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
        end
    

        [outputVec,statusFlag] = solve(ik,datapoint,guesses);
        % j1(t,1) = outputVec(1);
        % j2(t,1) = outputVec(2);
        % j3(t,1) = outputVec(3);
        % j4(t,1) = outputVec(4);
        % j5(t,1) = outputVec(5);
        % 
        % j1(t,1) = 0;
        % j2(t,1) = 0;
        % j3(t,1) = 0;
        % j4(t,1) = 0;
        % j5(t,1) = new_targets(t);
        % 
        
        j1(t,1) =  (com_motor_6(t)-500)*0.24;
        j2(t,1) =  (com_motor_5(t)-500)*0.24;
        j3(t,1) =  (com_motor_4(t)-500)*0.24;
        j4(t,1) =  (com_motor_3(t)-500)*0.24;
        j5(t,1) =  (com_motor_2(t)-500)*0.24;
  
 end
  %%2.2642 = vitesse max
  
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

for j=0:0   
    fprintf('Motor off is:%d\n',j);
    error1=m1;
    error2=m1;
    error3=m1;
    error4=m1;
    error5=m1;
    error6=m1;
   switch j
        case 1
            error1=m0;
        case 2
            error2=m0;
        case 3
            error3=m0;
        case 4
            error4=m0;
        case 5
            error5=m0;
        case 6
            error6=m0;
   end

    %on ajoute déjà les trajectoires cibles
    disp("----------------")
    disp("----------------")
    simOut = sim(model_name);
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


    j1_real =  (mov_6-500)*0.24;
j2_real = (mov_5-500)*0.24;
j3_real = (mov_4-500)*0.24;
j4_real = (mov_3-500)*0.24;
j5_real = (mov_2 -500)*0.24;

    [comparison_matrix, x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o, j5o,len_time_series); 
    [comparison_matrix_target, x_target, y_target, z_target] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series); 
    [comparison_matrix_real, x_real, y_real, z_real] = ForwardKinematic(j1_real, j2_real, j3_real, j4_real, j5_real,len_time_series); 
    matrix = comparison_matrix
    j1o = j1o/0.24 + 500;
    j2o = j2o/0.24 + 500;
    j3o = j3o/0.24 + 500;
    j4o = j4o/0.24 + 500;
    j5o = j5o/0.24 + 500;


    for i = 2:6
    
 
        plot_motor_movement_3(df, df_cmd, df_cmd_duration, ['motor_' num2str(i)],eval(['j' num2str(7-i) 'o']));
 
    end

end


x = 1:1000;
figure;
plot(x, j5o, 'b-', 'LineWidth', 2); % Plot real coordinates in blue
hold on; % Keep the current plot and add new plots
plot(x, new_targets, 'r-', 'LineWidth', 2); % Plot target coordinates in red
plot(x, position_degree_resampled-120, 'g-', 'LineWidth', 2);
% Add labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Plot of Real and Target Coordinates');





function [comparison_matrix, x, y ,z] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series)
    joint1_damping = 0;
    joint2_damping = 0;
    damp_pince = 1000; % damping coefficient for joints of the pince
    
    
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
    
        [outputVec,statusFlag] = solve(ik,targets);
        x(i,1) = outputVec(1);
        y(i,1) = outputVec(2);
        z(i,1) = outputVec(3);
    
        
    end
comparison_matrix = zeros(1000, 3);
comparison_matrix(:, 1) = x;
comparison_matrix(:, 2) = y;
comparison_matrix(:, 3) = z;
writematrix(comparison_matrix, "realised")

% Plot the motion in 3D space
figure;
plot3(comparison_matrix(:,1), comparison_matrix(:,2), comparison_matrix(:,3), 'g');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Comparison of Motions in 3D Space');
legend( 'Comparison motion')
grid on;




end

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
function [reference_positions, response_positions] = generate_positions(df, df_cmd, df_cmd_duration, motor, jo)

    command = df_cmd.(motor);
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

    starting_position = df{find(df.time_since_start<start_time(1), 1, 'last'), motor};
    points = [start_time(1), starting_position];
    points = [points; start_time(1)+duration(1)/1000, command(1)];
    for idx = 2:length(command)
        points = [points; start_time(idx), command(idx-1)];
        points = [points; start_time(idx)+duration(idx)/1000, command(idx)];
    end
    points = [points; df.time_since_start(end), command(end)];

    % Interpolate to get 1000 points for the reference curve
    x_values = points(:, 1);
    y_values = points(:, 2);
    interpolated_x = linspace(min(x_values), max(x_values), 1000);
    interpolated_y = interp1(x_values, y_values, interpolated_x);
    
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];

    % Calculate the actual response positions
    response_positions = interp1(df.time_since_start, df.(motor), interpolated_x);
end


function plot_motor_movement_2(df, df_cmd, df_cmd_duration, motor, jo)
    jo_resampled = interp1(linspace(0, 1, numel(jo)), jo, linspace(0, 1, numel(df.(motor))));
    figure;

    % Plot the response curve
    plot(df.time_since_start, df.(motor), 'LineWidth', 1.5, 'DisplayName', 'response');
    hold on;

    % Plot the reference curve
    command = df_cmd.(motor);
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

    starting_position = df{find(df.time_since_start<start_time(1), 1, 'last'), motor};
    points = [start_time(1), starting_position];
    points = [points; start_time(1)+duration(1)/1000, command(1)];
    for idx = 2:length(command)
        points = [points; start_time(idx), command(idx-1)];
        points = [points; start_time(idx)+duration(idx)/1000, command(idx)];
    end
    points = [points; df.time_since_start(end), command(end)];

    plot(points(:,1), points(:,2), '-or', 'DisplayName', 'reference');

    % Resample jo to match the timestamps of df


    % Plot the jo curve
    plot(df.time_since_start, jo_resampled, 'LineWidth', 1.5, 'DisplayName', 'jo');

    hold off;

    legend;
    xlabel('time since start (seconds)');
    ylabel('position (degrees)');
    title([motor ' movement']);
    grid on;
end


function plot_motor_movement_3(df, df_cmd, df_cmd_duration, motor, jo)
    figure;
    plot(df.time_since_start, df.(motor), 'LineWidth', 1.5, 'DisplayName', 'response');

    command = df_cmd.(motor);
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

    starting_position = df{find(df.time_since_start<start_time(1), 1, 'last'), motor};
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
    hold on;
    plot(points(:,1), points(:,2), '-or', 'DisplayName', 'reference');
    plot(interpolated_x, interpolated_y, 'DisplayName', 'interpolated reference');
    plot(interpolated_x, jo', 'DisplayName', 'jo')
    hold off;

    legend;
    xlabel('time since start (seconds)');
    ylabel('position (degrees)');
    title([motor ' movement']);
    grid on;
    
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];
end
