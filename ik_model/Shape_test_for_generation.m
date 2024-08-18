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

use_case = "5D";



%% preparing data for full trajectory test

%specify source files for the trajectory

if use_case == "5D"
len_time_series = 1000; % because most commands don't contain point-by-point commands, this has to be specified by the user


%the following variables represent three provided .csv tables : the real
%response df, the real command points df_cmd, and the associated durations
%for each as df_cmd_duration.
df = readtable('trajectory_monitoring_position1.csv');

df_cmd = readtable('trajectory_monitoring_cmd1.csv');
df_cmd_duration = readtable('trajectory_monitoring_cmd_duration1.csv');

% Calculate time since start for both dataframes
df.time_since_start = df.timestamp - df.timestamp(1);
disp(df.time_since_start)
df_cmd.time_since_start = max(0,df_cmd.timestamp - df.timestamp(1));


% Convert position values to the axis system used in our simulations
df_deg = df{:, 2:end} / 1000 * 240;
df_cmd_deg = df_cmd{:, 2:end} / 1000 * 240;
   
end 
%for this section xyz_targets is simply a .csv containing positions along
%each axis for the target positions. The code will perform IK-FK operations
%to simulate. Comparison with real response not yet supported.

if use_case == "3D"

xyz_targets = load('target_traj.csv');
len_time_series=length(xyz_targets);

end


%% Commands 

%this section calls generate_positions to turn the provided points into
%exploitable time series. See function definitions for details on the
%operations.

[com_motor_6, mov_6] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(6)],len_time_series);
[com_motor_5, mov_5] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(5)],len_time_series);
[com_motor_4, mov_4] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(4)],len_time_series);
[com_motor_3, mov_3] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(3)],len_time_series);
[com_motor_2, mov_2] =  generate_positions(df, df_cmd, df_cmd_duration, ['motor_' num2str(2)],len_time_series);
% 

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
        j1(t,1) =  (com_motor_6(t)-500)*0.24;
        j2(t,1) =  (com_motor_5(t)-500)*0.24;
        j3(t,1) =  (com_motor_4(t)-500)*0.24;
        j4(t,1) =  (com_motor_3(t)-500)*0.24;
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

%% Results

%the following matrices serve to represent in a 3D plane the outputs. This
%allows comparisons of the actual trajectories, which may be required in
%lieu of comparing motor angular positions.
    disp(j1o)
    disp(j1_real)
    [comparison_matrix, x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o, j5o,len_time_series); 
    [comparison_matrix_target, x_target, y_target, z_target] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series); 
    [comparison_matrix_real, x_real, y_real, z_real] = ForwardKinematic(j1_real, j2_real, j3_real, j4_real, j5_real,len_time_series); 
   
    real_datapoint = [comparison_matrix_target, comparison_matrix_real];
    simulated_datapoint = [comparison_matrix_real, comparison_matrix];

    j1o = j1o/0.24 + 500;
    j2o = j2o/0.24 + 500;
    j3o = j3o/0.24 + 500;
    j4o = j4o/0.24 + 500;
    j5o = j5o/0.24 + 500;


    for i = 2:6
    
 
        plot_motor_movement_3(df, df_cmd, df_cmd_duration, ['motor_' num2str(i)],eval(['j' num2str(7-i) 'o']));
 
    end

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
    counter = 0
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
figure;
plot3(comparison_matrix(:,1), comparison_matrix(:,2), comparison_matrix(:,3), 'g');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Comparison of Motions in 3D Space');
legend( 'Comparison motion')
grid on;

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
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);
index = find(df.time_since_start < start_time(1), 1, 'last');

% Check if the index is empty and assign the starting position accordingly
if isempty(index)
    starting_position = df{1, motor};
    disp('empty index')
else
    starting_position = df{index, motor};
end
    disp('starting_position')
    disp(starting_position)
    disp('after starting_position')
    points = [start_time(1), starting_position];
    disp(size(points))
    points = [points; start_time(1)+duration(1)/len_time_series, command(1)];
    for idx = 2:length(command)
        points = [points; start_time(idx), command(idx-1)];
        points = [points; start_time(idx)+duration(idx)/len_time_series, command(idx)];
    end
    points = [points; df.time_since_start(end), command(end)];

    % Interpolate linearly along the given motor's positions to get len_time_series points for the reference curve
    x_values = points(:, 1);
    y_values = points(:, 2);
    interpolated_x = linspace(min(x_values), max(x_values), len_time_series);
    interpolated_y = interp1(x_values, y_values, interpolated_x);
    
    % Return the positions of the reference curve
    reference_positions = [interpolated_x', interpolated_y'];

    % Interpolate the response to len_time_series
    disp('real traj dump')
     disp(df.time_since_start)
    disp(df.(motor))
    disp(interpolated_x)
    disp('end of real traj dump')
    response_positions = interp1(df.time_since_start, df.(motor), interpolated_x);

end



function plot_motor_movement_3(df, df_cmd, df_cmd_duration, motor, jo)
    figure;
    plot(df.time_since_start, df.(motor), 'LineWidth', 1.5, 'DisplayName', 'response');

    command = df_cmd.(motor);
    start_time = df_cmd.time_since_start;
    duration = df_cmd_duration.(motor);

    % Find the index where df.time_since_start is less than start_time(1)
index = find(df.time_since_start < start_time(1), 1, 'last');

% Check if the index is empty and assign the starting position accordingly
if isempty(index)
    starting_position = df{1, motor};
    disp('empty index')
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
