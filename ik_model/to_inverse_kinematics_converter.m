post_ik_data_real_cell = {}
chosen_dataset = real_cell_dataset
for i = 1: 5 %numel(chosen_dataset)
    data = chosen_dataset{i}'


    joint1_damping = 10000;
joint2_damping = 100000;
joint2_damping = 100000;
joint2_damping = 100000;
damp_pince = 1000; % damping coefficient for joints of the pince

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
guesses = [0.1,0.1,0.1,0.1,0.1];
addInitialGuessVariables(ik,guessesIDs);


j1 = zeros(1000,1);
j2 = zeros(1000,1);
j3 = zeros(1000,1);
j4 = zeros(1000,1);
j5 = zeros(1000,1);
T = 10; % period
spline = zeros(1000,3);
for t = 1:1000
                %t_echantillon=t/500;
                datapoint_command =[data(1,t), data(2,t),data(3,t)];
                assignin('base','datapoint', datapoint_command)
                % datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                %datapoint = [0+k*0.1*cos(t/100*(2*pi/T)),0+k*0.1*sin(t/100*(2*pi/T)),0.15+k*0.1*(t/100/T)];
                spline(t,:)  = datapoint_command;
                assignin('base','spline', spline)
                targets(t,:) = datapoint_command;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1_com(t-1,1),j2_com(t-1,1),j3_com(t-1,1),j4_com(t-1,1),j5_com(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint_command, guesses);
                j1_com(t,1) = outputVec(1);
                j2_com(t,1) = outputVec(2);
                j3_com(t,1) = outputVec(3);
                j4_com(t,1) = outputVec(4);
                j5_com(t,1) = outputVec(5);



                                %t_echantillon=t/500;
                datapoint_response =[data(4,t), data(5,t),data(6,t)];
                assignin('base','datapoint', datapoint_response)
                % datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                %datapoint = [0+k*0.1*cos(t/100*(2*pi/T)),0+k*0.1*sin(t/100*(2*pi/T)),0.15+k*0.1*(t/100/T)];
                spline(t,:)  = datapoint_response;
                assignin('base','spline', spline)
                targets(t,:) = datapoint_response;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1_resp(t-1,1),j2_resp(t-1,1),j3_resp(t-1,1),j4_resp(t-1,1),j5_resp(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint_response, guesses);
                j1_resp(t,1) = outputVec(1);
                j2_resp(t,1) = outputVec(2);
                j3_resp(t,1) = outputVec(3);
                j4_resp(t,1) = outputVec(4);
                j5_resp(t,1) = outputVec(5);
end
post_ik_data_real_cell{i} = [j1_com, j2_com, j3_com, j4_com, j1_resp, j2_resp , j3_resp   , j4_resp ]'
end
post_ik_data_real_cell = post_ik_data_real_cell'


%% with urdf-extracted rigidtree

urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf'
% Import the robot model as a rigid body tree
robot = importrobot(urdfFilePath);


robot.DataFormat = 'row';
showdetails(robot);




ik = inverseKinematics('RigidBodyTree', robot);
weights = [1, 1, 1, 1, 1, 1];
guesses = [0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0];
% Specify the end-effector
endEffector = 'link5'






post_ik_data_real_cell = {}
chosen_dataset = real_cell_dataset
j1_com = zeros(1000,1);
j2_com = zeros(1000,1);
j3_com = zeros(1000,1);
j4_com = zeros(1000,1);
j5_com = zeros(1000,1);
T = 10; % period
spline = zeros(1000,3);
chosen_dataset = real_cell_dataset

for i = 1:numel(chosen_dataset)
data = chosen_dataset{i}'
for t = 1:length(data)
                %t_echantillon=t/500;
                datapoint_command =[data(1,t), data(2,t),data(3,t)];
                targetOrientation = eye(3); 
                targets(t,:) = datapoint_command;
                pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
                
                
                if t>1 
                    guesses = [j1_com(t-1,1),j2_com(t-1,1),j3_com(t-1,1),j4_com(t-1,1),j5_com(t-1,1),o6,o7,o8,o9,o10,o11];
                    assignin('base','guesses', guesses)
                end
            
                               [output, ~]=  ik(endEffector,pose,weights,guesses);

                j1_com(t,1) = output(1);
                j2_com(t,1) = output(2);
                j3_com(t,1) = output(3);
                j4_com(t,1) = output(4);
                j5_com(t,1) = output(5);
                o6 =output(6);
                o7 = output(6);
                o8 = output(6);
                o9 = output(6);
                o10 = output(6);
                o11 = output(6);




                datapoint_response =[data(4,t), data(5,t),data(6,t)];
                targetOrientation = eye(3); 
                targets(t,:) = datapoint_response;
                pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);
        
                
                
                if t>1 
                    guesses = [j1_resp(t-1,1),j2_resp(t-1,1),j3_resp(t-1,1),j4_resp(t-1,1),j5_resp(t-1,1),o6,o7,o8,o9,o10,o11];
                    assignin('base','guesses', guesses)
                end
            
                               [output, ~]=  ik(endEffector,pose,weights,guesses);

                j1_resp(t,1) = output(1);
                j2_resp(t,1) = output(2);
                j3_resp(t,1) = output(3);
                j4_resp(t,1) = output(4);
                j5_resp(t,1) = output(5);
                o6 =output(6);
                o7 = output(6);
                o8 = output(6);
                o9 = output(6);
                o10 = output(6);
                o11 = output(6);



         
end
post_ik_data_real_cell{i} = [j1_com'; j2_com'; j3_com'; j4_com';j1_resp'; j2_resp'; j3_resp'; j4_resp']
    figure;
plot_data = post_ik_data_real_cell{i};
    % Plot lines 1 and 5
    subplot(4, 1, 1); % Create the first subplot
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command'); % Plot the first line in red
    hold on; % Hold the plot for adding more data
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response'); % Plot the fifth line in blue
    hold off;
    legend; % Show the legend
    title(sprintf('Iteration %d: Lines 1 and 5', i)); % Title for the subplot
    xlabel('Sample'); % Label for the x-axis
    ylabel('Value'); % Label for the y-axis

    % Plot lines 2 and 6
    subplot(4, 1, 2); % Create the second subplot
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command'); % Plot the second line in red
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response'); % Plot the sixth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot lines 3 and 7
    subplot(4, 1, 3); % Create the third subplot
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command'); % Plot the third line in red
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response'); % Plot the seventh line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot lines 4 and 8
    subplot(4, 1, 4); % Create the fourth subplot
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command'); % Plot the fourth line in red
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response'); % Plot the eighth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');
    
end
post_ik_data_real_cell = post_ik_data_real_cell'

%%

%% with urdf-extracted rigidtree

urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
% Import the robot model as a rigid body tree
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; % IK weights
guesses = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector

% Define maximum allowable change in joint angles (radians)
maxDeltaAngle = 0.09; % Adjust this value as needed

% Define damping factor (0 < dampingFactor <= 1)
dampingFactor = 0.5; % Adjust this value based on your needs

% Initialize variables
post_ik_data_real_cell = {};
chosen_dataset = cell_array%real_cell_dataset; % Use the chosen dataset
chosen_len = 100
j1_com = zeros(chosen_len,1);
j2_com = zeros(chosen_len,1);
j3_com = zeros(chosen_len,1);
j4_com = zeros(chosen_len,1);
j5_com = zeros(chosen_len,1);

j1_resp = zeros(chosen_len,1);
j2_resp = zeros(chosen_len,1);
j3_resp = zeros(chosen_len,1);
j4_resp = zeros(chosen_len,1);
j5_resp = zeros(chosen_len,1);

o6 = 0; o7 = 0; o8 = 0; o9 = 0; o10 = 0; o11 = 0;

for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i}';
    data = data(:,1:10:end)
    
    for t = 1:length(data)
        % Command datapoint
        datapoint_command = [data(1,t), data(2,t), data(3,t)];
        targetOrientation = eye(3); 
        pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
        if t > 1
            guesses = [j1_com(t-1,1), j2_com(t-1,1), j3_com(t-1,1), j4_com(t-1,1), j5_com(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [output, ~] = ik(endEffector, pose, weights, guesses);
        
        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Limit the change in joint angles
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store results
        j1_com(t,1) = output(1);
        j2_com(t,1) = output(2);
        j3_com(t,1) = output(3);
        j4_com(t,1) = output(4);
        j5_com(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);

        % Response datapoint
        datapoint_response = [data(4,t), data(5,t), data(6,t)];
        pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);

        if t > 1
            guesses = [j1_resp(t-1,1), j2_resp(t-1,1), j3_resp(t-1,1), j4_resp(t-1,1), j5_resp(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for response
        [output, ~] = ik(endEffector, pose, weights, guesses);

        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Limit the change in joint angles
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)] + deltaAngles;
        end
        
        % Store results
        j1_resp(t,1) = output(1);
        j2_resp(t,1) = output(2);
        j3_resp(t,1) = output(3);
        j4_resp(t,1) = output(4);
        j5_resp(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);
    end
    
    % Store the post-processed data
    post_ik_data_real_cell{i} = [j1_com'; j2_com'; j3_com'; j4_com'; j5_com'; j1_resp'; j2_resp'; j3_resp'; j4_resp'; j5_resp'];

    % Plot the data
    figure;
    plot_data = post_ik_data_real_cell{i};
    
    subplot(4, 1, 1);
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command');
    hold on;
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 1 and 5', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 2);
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command');
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 3);
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command');
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 4);
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command');
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i));
    xlabel('Sample');
    ylabel('Value');
end

post_ik_data_real_cell = post_ik_data_real_cell';


%% single 



urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
% Import the robot model as a rigid body tree
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; % IK weights
guesses = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector
                    
% Define maximum allowable change in joint angles (radians)
maxDeltaAngle = 0.09; % Adjust this value as needed

% Define damping factor (0 < dampingFactor <= 1)
dampingFactor = 0.5; % Adjust this value based on your needs

% Initialize variables
post_ik_data_real_cell_sim = {};
chosen_dataset = new_cell_array; % First dataset

chosen_len = 50; % Adjust as necessary
j1_com = zeros(chosen_len,1);
j2_com = zeros(chosen_len,1);
j3_com = zeros(chosen_len,1);
j4_com = zeros(chosen_len,1);
j5_com = zeros(chosen_len,1);

j1_resp = zeros(chosen_len,1);
j2_resp = zeros(chosen_len,1);
j3_resp = zeros(chosen_len,1);
j4_resp = zeros(chosen_len,1);
j5_resp = zeros(chosen_len,1);
o6 = 0; o7 = 0; o8 = 0; o9 = 0; o10 = 0; o11 = 0;
for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i};
    data = data(:,1:20:end);

 % Define the number of motors and the allowed deviation percentage
numMotors = 5; % Number of motors (j1 to j5)
deviationPercentage = 0.20; % 20% deviation
    ignoreMotor = randi(numMotors);
    fprintf('Ignoring motor j%d for deviation check\n', ignoreMotor);
for t = 1:length(data)
    % --------- First Dataset (chosen_dataset) ---------
    % Command datapoint
    datapoint_command = [data(1,t), data(2,t), data(3,t)];
    targetOrientation = eye(3); 
    pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
    
    if t > 1
        guesses = [j1_com(t-1,1), j2_resp(t-1,1), j3_com(t-1,1), j4_com(t-1,1), j5_com(t-1,1), o6, o7, o8, o9, o10, o11];
    end
    
    % Compute IK
    [output, ~] = ik(endEffector, pose, weights, guesses);
    
    if t > 1
        % Apply damping to the joint movements
        dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
        
        % Calculate the change in joint angles
        deltaAngles = dampedOutput - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
        
        % Limit the change in joint angles
        deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
        
        % Update the joint angles with the constrained changes
        output(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
    end
    
    % Store results
    j1_com(t,1) = output(1);
    j2_com(t,1) = output(2);
    j3_com(t,1) = output(3);
    j4_com(t,1) = output(4);
    j5_com(t,1) = output(5);
    o6 = output(6);
    o7 = output(6);
    o8 = output(6);
    o9 = output(6);
    o10 = output(6);
    o11 = output(6);

    % Response datapoint
    datapoint_response = [data(4,t), data(5,t), data(6,t)];
    pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);

    if t > 1
        guesses = [j1_resp(t-1,1), j2_resp(t-1,1), j3_resp(t-1,1), j4_resp(t-1,1), j5_resp(t-1,1), o6, o7, o8, o9, o10, o11];
    end
    
    % Compute IK for response
    [output, ~] = ik(endEffector, pose, weights, guesses);

    % Randomly choose a motor to ignore

    
    if t > 1
        % Apply damping to the joint movements
        dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
        
        % Calculate the deviation and enforce constraints
        
    end
    
    % Store results
    j1_resp(t,1) = output(1);
    j2_resp(t,1) = output(2);
    j3_resp(t,1) = output(3);
    j4_resp(t,1) = output(4);
    j5_resp(t,1) = output(5);
    o6 = output(6);
    o7 = output(6);
    o8 = output(6);
    o9 = output(6);
    o10 = output(6);
    o11 = output(6);

end
    % Store the post-processed data
    post_ik_data_real_cell_sim{i} = [j1_com'; j2_com'; j3_com'; j4_com'; j1_com'; j2_resp'; j3_resp'; j4_resp'];
    % Plot the data
    figure;
    plot_data = post_ik_data_real_cell_sim{i};

    subplot(4, 1, 1);
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 1 and 5', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 2);
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 3);
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 4);
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i));
    xlabel('Sample');
    ylabel('Value');
end

post_ik_data_real_cell_sim = post_ik_data_real_cell_sim';
numel(post_ik_data_real_cell_sim)


%% real and simulated plotted together



urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
% Import the robot model as a rigid body tree
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1, 1, 1, 0, 0, 0]; % IK weights
guesses = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector
                    
% Define maximum allowable change in joint angles (radians)
maxDeltaAngle = 0.09; % Adjust this value as needed

% Define damping factor (0 < dampingFactor <= 1)
dampingFactor = 0.5; % Adjust this value based on your needs

% Initialize variables
post_ik_data_real_cell = {};
chosen_dataset = real_cell_dataset; % First dataset
cell_array = cell_array; % Second dataset

chosen_len = 50; % Adjust as necessary
j1_com = zeros(chosen_len,1);
j2_com = zeros(chosen_len,1);
j3_com = zeros(chosen_len,1);
j4_com = zeros(chosen_len,1);
j5_com = zeros(chosen_len,1);

j1_resp = zeros(chosen_len,1);
j2_resp = zeros(chosen_len,1);
j3_resp = zeros(chosen_len,1);
j4_resp = zeros(chosen_len,1);
j5_resp = zeros(chosen_len,1);

j1_com2 = zeros(chosen_len,1); % For the second dataset
j2_com2 = zeros(chosen_len,1);
j3_com2 = zeros(chosen_len,1);
j4_com2 = zeros(chosen_len,1);
j5_com2 = zeros(chosen_len,1);

j1_resp2 = zeros(chosen_len,1); % For the second dataset
j2_resp2 = zeros(chosen_len,1);
j3_resp2 = zeros(chosen_len,1);
j4_resp2 = zeros(chosen_len,1);
j5_resp2 = zeros(chosen_len,1);

o6 = 0; o7 = 0; o8 = 0; o9 = 0; o10 = 0; o11 = 0;

for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i}';
    data = data(:,1:20:end);
    data(1,:) = data(1,:);
    data2 = cell_array{i}'; % For the second dataset
    data2 = data2(:,1:20:end);
    data2(1,:) = data2(1,:);
    for t = 1:length(data)
        % --------- First Dataset (chosen_dataset) ---------
        % Command datapoint
        datapoint_command = [data(1,t), data(2,t), data(3,t)];
        targetOrientation = eye(3); 
        pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
        if t > 1
            guesses = [j1_com(t-1,1), j2_com(t-1,1), j3_com(t-1,1), j4_com(t-1,1), j5_com(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [output, ~] = ik(endEffector, pose, weights, guesses);
        
        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Limit the change in joint angles
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store results
        j1_com(t,1) = output(1);
        j2_com(t,1) = output(2);
        j3_com(t,1) = output(3);
        j4_com(t,1) = output(4);
        j5_com(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);

        % Response datapoint
        datapoint_response = [data(4,t), data(5,t), data(6,t)];
        pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);

        if t > 1
            guesses = [j1_resp(t-1,1), j2_resp(t-1,1), j3_resp(t-1,1), j4_resp(t-1,1), j5_resp(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for response
        [output, ~] = ik(endEffector, pose, weights, guesses);

        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Limit the change in joint angles
            deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)] + deltaAngles;
        end
        
        % Store results
        
        j1_resp(t,1) =  output(1);
        j2_resp(t,1) = output(2);
        j3_resp(t,1) = output(3);
        j4_resp(t,1) = output(4);
        j5_resp(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);

        % --------- Second Dataset (cell_array) ---------
        % Command datapoint
        datapoint_command2 = [data2(1,t), data2(2,t), data2(3,t)];
        targetOrientation2 = eye(3); 
        pose2 = trvec2tform(datapoint_command2) * rotm2tform(targetOrientation2);
        
        if t > 1
            guesses = [j1_com2(t-1,1), j2_com2(t-1,1), j3_com2(t-1,1), j4_com2(t-1,1), j5_com2(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for second dataset
        [output2, ~] = ik(endEffector, pose2, weights, guesses);
        
        if t > 1
            % Apply damping to the joint movements
            dampedOutput2 = dampingFactor * output2(1:5) + (1 - dampingFactor) * [j1_com2(t-1), j2_com2(t-1), j3_com2(t-1), j4_com2(t-1), j5_com2(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles2 = dampedOutput2 - [j1_com2(t-1), j2_com2(t-1), j3_com2(t-1), j4_com2(t-1), j5_com2(t-1)];
            
            % Limit the change in joint angles
            deltaAngles2 = max(min(deltaAngles2, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output2(1:5) = [j1_com2(t-1), j2_com2(t-1), j3_com2(t-1), j4_com2(t-1), j5_com2(t-1)] + deltaAngles2;
        end
        
        % Store results
        j1_com2(t,1) = output2(1);
        j2_com2(t,1) = output2(2);
        j3_com2(t,1) = output2(3);
        j4_com2(t,1) = output2(4);
        j5_com2(t,1) = output2(5);

        % Response datapoint for second dataset
        datapoint_response2 = [data2(4,t), data2(5,t), data2(6,t)];
        pose2 = trvec2tform(datapoint_response2) * rotm2tform(targetOrientation2);

        if t > 1
            guesses = [j1_resp2(t-1,1), j2_resp2(t-1,1), j3_resp2(t-1,1), j4_resp2(t-1,1), j5_resp2(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for response (second dataset)
        [output2, ~] = ik(endEffector, pose2, weights, guesses);

        if t > 1
            % Apply damping to the joint movements
            dampedOutput2 = dampingFactor * output2(1:5) + (1 - dampingFactor) * [j1_resp2(t-1), j2_resp2(t-1), j3_resp2(t-1), j4_resp2(t-1), j5_resp2(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles2 = dampedOutput2 - [j1_resp2(t-1), j2_resp2(t-1), j3_resp2(t-1), j4_resp2(t-1), j5_resp2(t-1)];
            
            % Limit the change in joint angles
            deltaAngles2 = max(min(deltaAngles2, maxDeltaAngle), -maxDeltaAngle);
            
            % Update the joint angles with the constrained changes
            output2(1:5) = [j1_resp2(t-1), j2_resp2(t-1), j3_resp2(t-1), j4_resp2(t-1), j5_resp2(t-1)] + deltaAngles2;
        end
        
        % Store results
        j1_resp2(t,1) = output2(1);
        j2_resp2(t,1) = output2(2);
        j3_resp2(t,1) = output2(3);
        j4_resp2(t,1) = output2(4);
        j5_resp2(t,1) = output2(5);
    end
    
    % Store the post-processed data
    post_ik_data_real_cell{i} = [j1_com'; j2_com'; j3_com'; j4_com'; j1_com'; j2_resp'; j3_resp'; j4_resp'];
    post_ik_data_real_cell2{i} = [j1_com2'; j2_com2'; j3_com2'; j4_com2'; j1_com2'; j2_resp2'; j3_resp2'; j4_resp2'];

    % Plot the data
    figure;
    plot_data = post_ik_data_real_cell{i};
    plot_data2 = post_ik_data_real_cell2{i};

    subplot(4, 1, 1);
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    plot(plot_data2(1, :), 'g', 'DisplayName', 'Command (Dataset 2)');
    plot(plot_data2(5, :), 'k', 'DisplayName', 'Response (Dataset 2)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 1 and 5', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 2);
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    plot(plot_data2(2, :), 'g', 'DisplayName', 'Command (Dataset 2)');
    plot(plot_data2(6, :), 'k', 'DisplayName', 'Response (Dataset 2)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 3);
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    plot(plot_data2(3, :), 'g', 'DisplayName', 'Command (Dataset 2)');
    plot(plot_data2(7, :), 'k', 'DisplayName', 'Response (Dataset 2)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i));
    xlabel('Sample');
    ylabel('Value');

    subplot(4, 1, 4);
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    plot(plot_data2(4, :), 'g', 'DisplayName', 'Command (Dataset 2)');
    plot(plot_data2(8, :), 'k', 'DisplayName', 'Response (Dataset 2)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i));
    xlabel('Sample');
    ylabel('Value');
end

post_ik_data_real_cell = post_ik_data_real_cell';
post_ik_data_real_cell2 = post_ik_data_real_cell2';


%% max deviation controller

% Initialize variables
post_ik_data_real_cell_sim = {};
chosen_dataset = new_cell_array; % First dataset

chosen_len = 50; % Adjust as necessary
j1_com = zeros(chosen_len,1);
j2_com = zeros(chosen_len,1);
j3_com = zeros(chosen_len,1);
j4_com = zeros(chosen_len,1);
j5_com = zeros(chosen_len,1);

j1_resp = zeros(chosen_len,1);
j2_resp = zeros(chosen_len,1);
j3_resp = zeros(chosen_len,1);
j4_resp = zeros(chosen_len,1);
j5_resp = zeros(chosen_len,1);
o6 = 0; o7 = 0; o8 = 0; o9 = 0; o10 = 0; o11 = 0;

% Tuning parameters
maxDeltaAngleRelaxed = 0.15; % Larger allowed deviation for the problematic joint

for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i};
    data = data(:,1:10:end);

    for t = 1:length(data)
        % --------- First Dataset (chosen_dataset) ---------
        % Command datapoint
        datapoint_command = [data(1,t), data(2,t), data(3,t)];
        targetOrientation = eye(3); 
        pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
        
        if t > 1
            guesses = [j1_com(t-1,1), j2_com(t-1,1), j3_com(t-1,1), j4_com(t-1,1), j5_com(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK
        [output, ~] = ik(endEffector, pose, weights, guesses);
        
        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
            
            % Identify the joint with the maximum deviation
            [~, maxDeviationIdx] = max(abs(deltaAngles));
            
            % Allow a larger deviation for the problematic joint
            for j = 1:5
                if j == maxDeviationIdx
                    deltaAngles(j) = max(min(deltaAngles(j), maxDeltaAngleRelaxed), -maxDeltaAngleRelaxed);
                else
                    deltaAngles(j) = max(min(deltaAngles(j), maxDeltaAngle), -maxDeltaAngle);
                end
            end
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
        end
        
        % Store results
        j1_com(t,1) = output(1);
        j2_com(t,1) = output(2);
        j3_com(t,1) = output(3);
        j4_com(t,1) = output(4);
        j5_com(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);

        % Response datapoint
        datapoint_response = [data(4,t), data(5,t), data(6,t)];
        pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);

        if t > 1
            guesses = [j1_resp(t-1,1), j2_resp(t-1,1), j3_resp(t-1,1), j4_resp(t-1,1), j5_resp(t-1,1), o6, o7, o8, o9, o10, o11];
        end
        
        % Compute IK for response
        [output, ~] = ik(endEffector, pose, weights, guesses);

        if t > 1
            % Apply damping to the joint movements
            dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Calculate the change in joint angles
            deltaAngles = dampedOutput - [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
            
            % Identify the joint with the maximum deviation
            [~, maxDeviationIdx] = max(abs(deltaAngles));
            
            % Allow a larger deviation for the problematic joint
            for j = 1:5
                if j == maxDeviationIdx
                    deltaAngles(j) = max(min(deltaAngles(j), maxDeltaAngleRelaxed), -maxDeltaAngleRelaxed);
                else
                    deltaAngles(j) = max(min(deltaAngles(j), maxDeltaAngle), -maxDeltaAngle);
                end
            end
            
            % Update the joint angles with the constrained changes
            output(1:5) = [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)] + deltaAngles;
        end
        
        % Store results
        j1_resp(t,1) = output(1);
        j2_resp(t,1) = output(2);
        j3_resp(t,1) = output(3);
        j4_resp(t,1) = output(4);
        j5_resp(t,1) = output(5);
        o6 = output(6);
        o7 = output(6);
        o8 = output(6);
        o9 = output(6);
        o10 = output(6);
        o11 = output(6);
    end
    
    % Store the post-processed data
    post_ik_data_real_cell_sim{i} = [j1_com'; j2_com'; j3_com'; j4_com'; j1_com'; j2_resp'; j3_resp'; j4_resp'];
        %Plot the data
    figure;
    plot_data = post_ik_data_real_cell_sim{i};

    subplot(4, 1, 1);
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 1 and 5', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 2);
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 3);
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 4);
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i));
    xlabel('Sample');
    ylabel('Value');
end

post_ik_data_real_cell_sim = post_ik_data_real_cell_sim';
numel(post_ik_data_real_cell_sim)

urdfFilePath  ='C:\Users\PC\MATLAB\Projects\armpi_final\pole_project\create_multibody_from_urdf\armpi_fpv\armpi_fpv.urdf';
% Import the robot model as a rigid body tree
robot = importrobot(urdfFilePath);
robot.DataFormat = 'row';
showdetails(robot);

% Set up inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1]; % IK weights
guesses = [0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0]; % Initial guesses
endEffector = 'link5'; % Specify the end-effector
                    
% Define maximum allowable change in joint angles (radians)
maxDeltaAngle = 0.09; % Adjust this value as needed

% Define damping factor (0 < dampingFactor <= 1)
dampingFactor = 0.5; % Adjust this value based on your needs

% Initialize variables
post_ik_data_real_cell_sim = {};
chosen_dataset = new_cell_array; % First dataset

chosen_len = 50; % Adjust as necessary
j1_com = zeros(chosen_len,1);
j2_com = zeros(chosen_len,1);
j3_com = zeros(chosen_len,1);
j4_com = zeros(chosen_len,1);
j5_com = zeros(chosen_len,1);

j1_resp = zeros(chosen_len,1);
j2_resp = zeros(chosen_len,1);
j3_resp = zeros(chosen_len,1);
j4_resp = zeros(chosen_len,1);
j5_resp = zeros(chosen_len,1);
o6 = 0; o7 = 0; o8 = 0; o9 = 0; o10 = 0; o11 = 0;
for i = 1:numel(chosen_dataset)
    data = chosen_dataset{i};
    data = data(:,1:20:end);

 % Define the number of motors and the allowed deviation percentage
numMotors = 5; % Number of motors (j1 to j5)
deviationPercentage = 0.20; % 20% deviation
    ignoreMotor = randi(numMotors);
    fprintf('Ignoring motor j%d for deviation check\n', ignoreMotor);
for t = 1:length(data)
    % --------- First Dataset (chosen_dataset) ---------
    % Command datapoint
    datapoint_command = [data(1,t), data(2,t), data(3,t)];
    targetOrientation = eye(3); 
    pose = trvec2tform(datapoint_command) * rotm2tform(targetOrientation);
    
    if t > 1
        guesses = [j1_com(t-1,1), j2_com(t-1,1), j3_com(t-1,1), j4_com(t-1,1), j5_com(t-1,1), o6, o7, o8, o9, o10, o11];
    end
    
    % Compute IK
    [output, ~] = ik(endEffector, pose, weights, guesses);
    
    if t > 1
        % Apply damping to the joint movements
        dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
        
        % Calculate the change in joint angles
        deltaAngles = dampedOutput - [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
        
        % Limit the change in joint angles
        deltaAngles = max(min(deltaAngles, maxDeltaAngle), -maxDeltaAngle);
        
        % Update the joint angles with the constrained changes
        output(1:5) = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)] + deltaAngles;
    end
    
    % Store results
    j1_com(t,1) = output(1);
    j2_com(t,1) = output(2);
    j3_com(t,1) = output(3);
    j4_com(t,1) = output(4);
    j5_com(t,1) = output(5);
    o6 = output(6);
    o7 = output(6);
    o8 = output(6);
    o9 = output(6);
    o10 = output(6);
    o11 = output(6);

    % Response datapoint
    datapoint_response = [data(4,t), data(5,t), data(6,t)];
    pose = trvec2tform(datapoint_response) * rotm2tform(targetOrientation);

    if t > 1
        guesses = [j1_resp(t-1,1), j2_resp(t-1,1), j3_resp(t-1,1), j4_resp(t-1,1), j5_resp(t-1,1), o6, o7, o8, o9, o10, o11];
    end
    
    % Compute IK for response
    [output, ~] = ik(endEffector, pose, weights, guesses);

    % Randomly choose a motor to ignore

    
    if t > 1
        % Apply damping to the joint movements
        dampedOutput = dampingFactor * output(1:5) + (1 - dampingFactor) * [j1_resp(t-1), j2_resp(t-1), j3_resp(t-1), j4_resp(t-1), j5_resp(t-1)];
        
        % Calculate the deviation and enforce constraints
        for i = 1:numMotors
            if i ~= ignoreMotor
                % Calculate deviation percentage
                nom = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
                denom = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
                deviation = abs(output(i) - nom(i)) / abs(denom(i));
                
                % Limit deviation to 20%
                if deviation > deviationPercentage
                    % Constrain the output within the acceptable range
                    temp_matrix_2 = [j1_com(t-1), j2_com(t-1), j3_com(t-1), j4_com(t-1), j5_com(t-1)];
                    output(i) = temp_matrix_2(i) + deviationPercentage * abs(temp_matrix_2(i)) * sign(output(i) - temp_matrix_2(i));
                end
            end
        end
    end
    
    % Store results
    j1_resp(t,1) = output(1);
    j2_resp(t,1) = output(2);
    j3_resp(t,1) = output(3);
    j4_resp(t,1) = output(4);
    j5_resp(t,1) = output(5);
    o6 = output(6);
    o7 = output(6);
    o8 = output(6);
    o9 = output(6);
    o10 = output(6);
    o11 = output(6);

end
    % Store the post-processed data
    post_ik_data_real_cell_sim{i} = [j1_com'; j2_com'; j3_com'; j4_com'; j1_com'; j2_resp'; j3_resp'; j4_resp'];
    % Plot the data
    figure;
    plot_data = post_ik_data_real_cell_sim{i};

    subplot(4, 1, 1);
    plot(plot_data(1, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(5, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 1 and 5', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 2);
    plot(plot_data(2, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(6, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 3);
    plot(plot_data(3, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(7, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i));
    xlabel('Sample');
    ylabel('Value');
    subplot(4, 1, 4);
    plot(plot_data(4, :), 'r', 'DisplayName', 'Command (Dataset 1)');
    hold on;
    plot(plot_data(8, :), 'b', 'DisplayName', 'Response (Dataset 1)');
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i));
    xlabel('Sample');
    ylabel('Value');
end

post_ik_data_real_cell_sim = post_ik_data_real_cell_sim';
numel(post_ik_data_real_cell_sim)
