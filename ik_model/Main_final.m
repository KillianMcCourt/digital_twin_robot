disp("It has begun")

%processing_simulation('trajectory_dataset_name','testing_if_dataset_generation_works_with_parser','circles',1,'interpolations',1)

%processing_simulation('testing_if_dataset_generation_works_with_parser',1,250,1,333,0,255,[1,9,11],'lol',0.89,0.90,1,1,2,'testmodel','testbase','testfollower','testtargetIDs','testoutputIDs','testguessesIDs',[1,1,1,1,1,1],1020)
processing_simulation('testing_if_dataset_generation_works_with_parser','circles',1,'lines',1,'interpolations',1)



function []= processing_simulation(varargin)
    %Parsing the intput arguments to enable the setup of default values
    
    % Create an input parser
    p = inputParser;
    
    % Define optional input arguments with default values
    addRequired(p, 'trajectory_dataset_name',@(x) ischar(x) || isstring(x));
    addOptional(p, 'circles', 0, @isnumeric);
    addOptional(p, 'circle_number', 100, @isnumeric);
    addOptional(p, 'lines', 0, @isnumeric);
    addOptional(p, 'line_number', 100, @isnumeric);
    addOptional(p, 'interpolations', 0, @isnumeric);
    addOptional(p, 'interpolation_number', 100, @isnumeric);
    addOptional(p, 'selection_erreures_moteur', [0,1,2,3,4,5,6,7,8,9,10,11,12], @isnumeric);
    addOptional(p, 'model_name', 'ik_model\main3_armpi_fpv', @(x) ischar(x) || isstring(x));
    addOptional(p, 'distance_charac_robot', 0.28, @isnumeric);%décrit la zone dans laquelle le robot peut agir, par défault prend la v aleur spécifique du cas de notre étude
    addOptional(p, 'speedcap', 0.2, @isnumeric);
    addOptional(p, 'joint1_damping', 0, @isnumeric);
    addOptional(p, 'joint2_damping', 0, @isnumeric);
    addOptional(p, 'damp_pince', 1000, @isnumeric);
    addOptional(p, 'mdl', "robot_model", @(x) ischar(x) || isstring(x));
    addOptional(p, 'base', "robot_model/World/W", @(x) ischar(x) || isstring(x));
    addOptional(p, 'follower', "robot_model/gripper_base/F", @(x) ischar(x) || isstring(x));
    addOptional(p, 'targetIDs', ["gripper_base.Translation.x";"gripper_base.Translation.y";...
        "gripper_base.Translation.z"], @(x) ischar(x) || isstring(x));
    addOptional(p, 'outputIDs', ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"], @(x) ischar(x) || isstring(x));  
    addOptional(p, 'guessesIDs', ["j1.Rz.q";"j2.Rz.q";"j3.Rz.q";"j4.Rz.q";"j5.Rz.q"], @(x) ischar(x) || isstring(x)); 
    addOptional(p, 'guesses', [3,3,3,3,3], @isnumeric);  
    %%% PAS ENCORE PLEINEMENT FONCTIONNEL, N AFFECTE QUE INTERPOLATION POUR L INSTANT
    addOptional(p, 'nb_points_traj', 1000, @isnumeric);
    
    % Parse the input arguments
    parse(p, varargin{:});

    % Retrieve the values
    disp("-------------------")
    trajectory_dataset_name = p.Results.trajectory_dataset_name;
    fprintf('Trajectory dataset name is:  %s.\n',trajectory_dataset_name);
    circles = p.Results.circles;
    circle_number = p.Results.circle_number;
    fprintf('Trajectory: circles Y/N: %d amount: %d.\n',circles,circle_number);
    lines = p.Results.lines;
    line_number = p.Results.line_number;
    fprintf('Trajectory: lines Y/N: %d amount: %d.\n',lines,line_number);
    interpolations = p.Results.interpolations;
    interpolation_number = p.Results.interpolation_number;
    fprintf('Trajectory: interpolation Y/N: %d amount: %d.\n',interpolations,interpolation_number);
    motorerrorselection = p.Results.selection_erreures_moteur;
    %fprintf('Les erreures moteur seront : %d m.\n',motorerrorselection);
    fprintf('Les erreures moteur seront : [%s%d]\n', sprintf('%d,', motorerrorselection(1:end-1)),motorerrorselection(end));
   
    %%déclaration globlale
    global model_name;
    model_name = p.Results.model_name;
    fprintf('Le modèle de robot simulé est : %s .\n',model_name);
    distance_charac_robot = p.Results.distance_charac_robot;
    fprintf('Le bras du robot pleinement étendu mesure: %f m.\n',distance_charac_robot);
    nb_points_traj = p.Results.nb_points_traj;
    fprintf('Le nombre de points par trajecoire est: %d points.\n',nb_points_traj);
    speedcap = p.Results.speedcap;
    fprintf('Le speedcap est: %d.\n',speedcap);
    disp("-------------------")

    %Creating the various trajectory datasets for the wanted shape types

    %Initialisation
    firsttype=0;
    secondtype=0;
    len_time_series=nb_points_traj;
    if circles || lines
        reduced_adapted_circle_set=CreateRandomCircleList(distance_charac_robot,circle_number);
        reduced_adapted_line_set=CreateRandomLineList(distance_charac_robot,line_number);
        reduced_adapted_shape_set=mergeStructures(reduced_adapted_circle_set,reduced_adapted_line_set);
        firsttype=1;
    end

    if interpolations
        reduced_adapted_interpolate_set= createInterpolate(interpolation_number, nb_points_traj);
        secondtype=1;
    end

    load_system(model_name);
    global joint1_damping;
    joint1_damping = p.Results.joint1_damping;
    assignin('base','joint1_damping',joint1_damping )
    fprintf('Le joint1_damping est: %d.\n',joint1_damping);
    global joint2_damping;
    joint2_damping = p.Results.joint2_damping;
    assignin('base','joint2_damping',joint2_damping )
    fprintf('Le joint1_damping est: %d.\n',joint2_damping);
    global damp_pince;
    damp_pince = p.Results.damp_pince;
    assignin('base','damp_pince', damp_pince)
    fprintf('Le damp_pince est: %d.\n',damp_pince);
    global mdl;
    mdl = p.Results.mdl;
    assignin('base','mdl', mdl)
    fprintf('Le modèle de robot est: %s.\n',mdl);
    load_system(mdl)
    global ik;
    ik = simscape.multibody.KinematicsSolver(mdl);
    assignin('base','ik', ik)
    global base;
    base = p.Results.base;
    assignin('base','base', base)
    fprintf('La base est: %s.\n',base);
    global follower;
    follower = p.Results.follower;
    assignin('base','follower', follower)
    fprintf('La follower est: %s.\n',follower);
    addFrameVariables(ik,"gripper_base","translation",base,follower);
    addFrameVariables(ik,"gripper_base","rotation",base,follower);
    global targetIDs;
    targetIDs = p.Results.targetIDs;
    assignin('base','targetIDs', targetIDs)
    fprintf('Les targetIDs sont: %s.\n',targetIDs);
    addTargetVariables(ik,targetIDs);
    global outputIDs;
    outputIDs = p.Results.outputIDs;
    assignin('base','outputIDs', outputIDs)
    fprintf('Les outputIDs sont: %s.\n',outputIDs);
    addOutputVariables(ik,outputIDs);
    global guessesIDs;
    guessesIDs = p.Results.guessesIDs;
    assignin('base','guessesIDs', guessesIDs)
    fprintf('Les guessesIDs sont: %s.\n',guessesIDs);
    global guesses;
    guesses = p.Results.guesses;
    assignin('base','guesses', guesses)
    fprintf('Les guesses sont : [%s%d]\n', sprintf('%d,', guesses(1:end-1)),guesses(end));
    addInitialGuessVariables(ik,guessesIDs);
    global datapoint;
    
    global simOut;
    
    %put all the variables in the workspace 

    
    %simul length: len_time_series/100= legnth of simulation in seconds

    scale_factor = randi([2, 4]);
    fprintf('Le scale facotr est: %d.\n', scale_factor);

    timescale=10/len_time_series;
    j1 = zeros(len_time_series,1);
    j2 = zeros(len_time_series,1);
    j3 = zeros(len_time_series,1);
    j4 = zeros(len_time_series,1);
    j5 = zeros(len_time_series,1);
    T = 10; % period
    spline = zeros(len_time_series,3);
    assignin('base','spline', spline)
    targets = zeros(len_time_series,3);
    assignin('base','targets', targets)

    %Creating dataset


    m0=[transpose(1:len_time_series), zeros(len_time_series, 1)];
    m1=[transpose(1:len_time_series), ones(len_time_series, 1)];

    if firsttype ~= 1 && secondtype ~= 1
        error('Please set either firsttype or secondtype to 1.');
    end
    
    dataset=[];
    if firsttype
        shapes_dict=reduced_adapted_shape_set;
        shapelist=fieldnames(shapes_dict);
        numberofshapes=numel(shapelist);
        fprintf('The number of fields is:%d\n',numberofshapes);
        for k = 1:numberofshapes
            if k==floor(numberofshapes/4)
                disp("------------------------")
                disp("K =1/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes/2)
                disp("------------------------")
                disp("K =1/2 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes*3/4)
                disp("------------------------")
                disp("K =3/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            
        
            shape=shapelist{k};
            disp("------------------------")
            fprintf('Shape name:%s\n',shape);
            disp("------------------------")
        
        
        
            for t = 1:len_time_series
                t_echantillon=t/500;
                datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                assignin('base','datapoint', datapoint)
                spline(t,:)  = datapoint;
                assignin('base','spline', spline)
                targets(t,:) = datapoint;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint,guesses);
                j1(t,1) = outputVec(1);
                j2(t,1) = outputVec(2);
                j3(t,1) = outputVec(3);
                j4(t,1) = outputVec(4);
                j5(t,1) = outputVec(5);
            end
                
            end_time_value_in_seconds= (len_time_series-1)*0.01;
        
            joint1_ts = timeseries(j1/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint1_ts', joint1_ts)
            joint2_ts = timeseries(j2/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint2_ts', joint2_ts)
            joint3_ts = timeseries(j3/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint3_ts', joint3_ts)
            joint4_ts = timeseries(j4/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint4_ts', joint4_ts)
            joint5_ts = timeseries(j5/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint5_ts', joint5_ts)
        
            for j=motorerrorselection   %il faut réparer les moteurs 4/5/6
                fprintf('Motor error is:%d\n',j);
                error1=m1;
                assignin('base','error1', error1)
                error2=m1;
                assignin('base','error2', error2)
                error3=m1;
                assignin('base','error3', error3)
                error4=m1;
                assignin('base','error4', error4)
                error5=m1;
                assignin('base','error5', error5)
                error6=m1;
                assignin('base','error6', error6)
        
                %%%% FOR RANDOM STOP-START BEHAVIOUR SIMULATION
        
                X = randi([1000, 1000]);
        
                % Step 2: Randomly select numbers greater than 50 that add up to X
                remainingX = X;
                selectedNumbers = [];
        
                while remainingX > 50
                    % Randomly select a number greater than 50
                    randomNumber = randi([51, remainingX]);
        
                    % Add the selected number to the list
                    selectedNumbers = [selectedNumbers, randomNumber];
        
                    % Update the remainingX
                    remainingX = remainingX - randomNumber;
                end
        
                % Specify the range
                lowerBound = 25;
                upperBound = 800;
                numPoints = numel(selectedNumbers);
                randomPoints = sort(randi([lowerBound, upperBound], 1, numPoints));
                totalPoints = len_time_series;
                pointsList = ones(1, totalPoints);
                pointsList(randomPoints) = 0;
        
                for i = 1:numPoints
                    startRange = randomPoints(i);
                    endRange = randomPoints(i) + selectedNumbers(i);
        
                    % Ensure the endRange does not exceed the total number of points
                    endRange = min(endRange, totalPoints);
        
                    % Set values to 0 in the specified range
                    pointsList(startRange:endRange) = 0;
                end
        
                % Assuming pointsList is already generated (as per the previous code)
        
                % Create a 1000x2 vector
                vectorMatrix = zeros(1000, 2);
        
                % Populate the first column with linear values from 1 to 1000
                vectorMatrix(:, 1) = (1:1000)';
        
                % Populate the second column with the values from pointsList
                vectorMatrix(:, 2) = pointsList;
        
    
                
        
                switch j
                    case 1
                        temp = error1;
                        error1=m0;
                        assignin('base','error1', error1)
                    case 2
                        error1 = temp;
                        assignin('base','error1', error1)
                        temp = error2;
                        error2=m0;
                        assignin('base','error2', error2)
                    case 3
                        error2 = temp;
                        assignin('base','error2', error2)
                        temp = error3;                    
                        error3=m0;
                        assignin('base','error3', error3)
                    case 4
                        error3 = temp;
                        assignin('base','error3', error3)
                        temp = joint1_ts.Data;                       
                        joint1_ts.Data = process_points(pointsList, joint1_ts.Data);
                        assignin('base','joint1_ts', joint1_ts)
                    case 5 
                        joint1_ts.Data = temp;
                        temp =  joint2_ts.Data;                       
                        joint2_ts.Data = process_points(pointsList, joint2_ts.Data);
                        assignin('base','joint2_ts', joint2_ts)
                    case 6
                        joint2_ts.Data = temp;
                        temp =  joint3_ts.Data;                       
                        joint3_ts.Data = process_points(pointsList, joint3_ts.Data);
                        assignin('base','joint3_ts', joint3_ts)
                    case 7  
                        joint3_ts.Data = temp;
                        temp =  joint1_ts.Data;                    
                        joint1_ts.Data = extend_trajectory(joint1_ts.Data, scale_factor);
                        assignin('base','joint1_ts', joint1_ts)
                    case 8
                        joint1_ts.Data = temp;
                        temp = joint2_ts.Data;
                        joint2_ts.Data = extend_trajectory(joint2_ts.Data, scale_factor);
                        assignin('base','joint2_ts', joint2_ts)
                    case 9
                        joint2_ts.Data = temp;
                        temp = joint3_ts.Data;
                        joint3_ts.Data = extend_trajectory(joint3_ts.Data, scale_factor);
                        assignin('base','joint3_ts', joint3_ts)
                    case 10
                        joint3_ts.Data = temp;
                        temp = joint1_ts.Data;                    
                        joint1_ts.Data = process_points_capped_speed(joint1_ts.Data, speedcap, timescale);
                        assignin('base','joint1_ts', joint1_ts)
                    case 11
                        joint1_ts.Data = temp;
                        temp = joint2_ts.Data;                    
                        joint2_ts.Data = process_points_capped_speed(joint2_ts.Data, speedcap, timescale);
                        assignin('base','joint2_ts', joint2_ts)
                    case 12
                        joint2_ts.Data = temp;
                        temp = joint3_ts.Data;                    
                        joint3_ts.Data = process_points_capped_speed(joint3_ts.Data, speedcap, timescale);
                        assignin('base','joint3_ts', joint3_ts)
                    case 13
                        error4=m0;
                        assignin('base','error4', error4)
                    case 14
                        error5=m0;
                        assignin('base','error4', error4)
                    case 15
                        error6=m0;
                        assignin('base','error4', error4)
            
                end
                dataset = [dataset, targets];
        
                %on ajoute déjà les trajectoires cibles
                disp("----------------")
                disp("----------------")
                
                simOut = sim(model_name);
                assignin('base','simOut', simOut)
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
        
                disp(size(j1o))
                [x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o, j5o,len_time_series); 
                jdatapoint = [x, y, z];

                dataset=[dataset,jdatapoint];
                %on ajoute jdatapoin au dataset, on a ainsi formé un bloc de six lignes associées à un point
            end
            %datapoint de labélisation j (pour une k ième forme donnée)
            %sera donnée par la mise bout a bout de 7*k i ème ligne de dataset
            %(cas pas d'erreure moteur)
            %et la 7*k+j ième ligne, chaque ligne étant x,y,z étudiés sur la
            %timeseries pour l'erreure moteur selecio
            %pour avoir une entrée de l'IA qui regarder l'effet  il faudra prendre
            %fprintf("The current size of the dataset is %s", mat2str(size(dataset)));
        end
            
        fprintf("The final size of the dataset1 is %s", mat2str(size(dataset)));
        v1=mat2str(size(dataset));
    end 

    if secondtype
        shapes_dict=reduced_adapted_interpolate_set;
        shapelist=fieldnames(shapes_dict);
        numberofshapes=numel(shapelist);
        fprintf('The number of fields is:%d\n',numberofshapes);
        dataset2=[];
        for k = 1:numberofshapes
            if k==floor(numberofshapes/4)
                disp("------------------------")
                disp("K =1/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            if k==floor(numberofshapes/2)
                disp("------------------------")
                disp("K =1/2 HAS BEEN REACHED")
                disp("------------------------")
            end
                if k==floor(numberofshapes*3/4)
                disp("------------------------")
                disp("K =3/4 HAS BEEN REACHED")
                disp("------------------------")
            end
            
        
            shape=shapelist{k};
            disp("------------------------")
            fprintf('Shape name:%s\n',shape);
            disp("------------------------")
        
            %disp("Equations de la forme");
            %disp(shapes_dict.(shape));
            x=shapes_dict.(shape).xcoords;
            y=shapes_dict.(shape).ycoords;
            z=shapes_dict.(shape).zcoords;
        
        
            for t = 1:len_time_series
                %t_echantillon=t/500;
                datapoint =[x(t), y(t),z(t)];
                assignin('base','datapoint', datapoint)
                % datapoint =[shapes_dict.(shape).xequation(t_echantillon), shapes_dict.(shape).yequation(t_echantillon), shapes_dict.(shape).zequation(t_echantillon)];
                %datapoint = [0+k*0.1*cos(t/100*(2*pi/T)),0+k*0.1*sin(t/100*(2*pi/T)),0.15+k*0.1*(t/100/T)];
                spline(t,:)  = datapoint;
                assignin('base','spline', spline)
                targets(t,:) = datapoint;
                assignin('base','targets', targets)
        
                
                
                if t>1 
                    guesses = [j1(t-1,1),j2(t-1,1),j3(t-1,1),j4(t-1,1),j5(t-1,1)];
                    assignin('base','guesses', guesses)
                end
            
        
                [outputVec,statusFlag] = solve(ik,datapoint,guesses);
                j1(t,1) = outputVec(1);
                j2(t,1) = outputVec(2);
                j3(t,1) = outputVec(3);
                j4(t,1) = outputVec(4);
                j5(t,1) = outputVec(5);
            end
    % %%%% new code for drawing command graph
    %      figure;
    %     plot3(x, y, z, 'LineWidth', 2);
    %     hold on;
    % 
    %     % Scatter plot with color gradient based on point index
    %     scatter3(x(1:10:end), y(1:10:end), z(1:10:end), 50, find(1:10:len_time_series), 'filled', 'MarkerEdgeColor', 'k');
    % 
    %     title(['Reference Trajectory ']);
    %     xlabel('X-axis');
    %     ylabel('Y-axis');
    %     zlabel('Z-axis');
    %     grid on;
    % 
    %     % Force MATLAB to update the figure window
    %     drawnow;
    % 
    % 
    % %%%%%



            end_time_value_in_seconds= (len_time_series-1)*0.01;

            joint1_ts = timeseries(j1/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint1_ts', joint1_ts)
            joint2_ts = timeseries(j2/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint2_ts', joint2_ts)
            joint3_ts = timeseries(j3/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint3_ts', joint3_ts)
            joint4_ts = timeseries(j4/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint4_ts', joint4_ts)
            joint5_ts = timeseries(j5/180*pi,0:0.01:end_time_value_in_seconds);
            assignin('base','joint5_ts', joint5_ts)
        
        
            for j=motorerrorselection   %il faut réparer les moteurs 4/5/6
                fprintf('Motor off is:%d\n',j);
                error1=m1;
                assignin('base','error1', error1)
                error2=m1;
                assignin('base','error2', error2)
                error3=m1;
                assignin('base','error3', error3)
                error4=m1;
                assignin('base','error4', error4)
                error5=m1;
                assignin('base','error5', error5)
                error6=m1;
                assignin('base','error6', error6)
        
                %%%% FOR RANDOM STOP-START BEHAVIOUR SIMULATION
        
                X = randi([400, 800]);
        
                % Step 2: Randomly select numbers greater than 50 that add up to X
                remainingX = X;
                selectedNumbers = [];
        
                while remainingX > 50
                    % Randomly select a number greater than 50
                    randomNumber = randi([51, remainingX]);
        
                    % Add the selected number to the list
                    selectedNumbers = [selectedNumbers, randomNumber];
        
                    % Update the remainingX
                    remainingX = remainingX - randomNumber;
                end
        
                % Specify the range
                lowerBound = 25;
                upperBound = 900;
                numPoints = numel(selectedNumbers);
                randomPoints = sort(randi([lowerBound, upperBound], 1, numPoints));
                totalPoints = 1000;
                pointsList = ones(1, totalPoints);
                pointsList(randomPoints) = 0;
        
                for i = 1:numPoints
                    startRange = randomPoints(i);
                    endRange = randomPoints(i) + selectedNumbers(i);
        
                    % Ensure the endRange does not exceed the total number of points
                    endRange = min(endRange, totalPoints);
        
                    % Set values to 0 in the specified range
                    pointsList(startRange:endRange) = 0;
                end
        
                % Assuming pointsList is already generated (as per the previous code)
        
                % Create a 1000x2 vector
                vectorMatrix = zeros(1000, 2);
        
                % Populate the first column with linear values from 1 to 1000
                vectorMatrix(:, 1) = (1:1000)';
        
                % Populate the second column with the values from pointsList
                vectorMatrix(:, 2) = pointsList;
        
        
        
                %%

                %pointsList = [ones(100, 1); zeros(500, 1);ones(400, 1)];
                switch j
                    case 1
                        temp = error1;
                        error1=m0;
                        assignin('base','error1', error1)
                    case 2
                        error1 = temp;
                        assignin('base','error1', error1)
                        temp = error2;
                        error2=m0;
                        assignin('base','error2', error2)
                    case 3
                        error2 = temp;
                        assignin('base','error2', error2)
                        temp = error3;                    
                        error3=m0;
                        assignin('base','error3', error3)
                    case 4
                        error3 = temp;
                        assignin('base','error3', error3)
                        temp = joint1_ts.Data;                       
                        joint1_ts.Data = process_points(pointsList, joint1_ts.Data);
                        assignin('base','joint1_ts', joint1_ts)
                    case 5 
                        joint1_ts.Data = temp;
                        temp =  joint2_ts.Data;                       
                        joint2_ts.Data = process_points(pointsList, joint2_ts.Data);
                        assignin('base','joint2_ts', joint2_ts)
                    case 6
                        joint2_ts.Data = temp;
                        temp =  joint3_ts.Data;                       
                        joint3_ts.Data = process_points(pointsList, joint3_ts.Data);
                        assignin('base','joint3_ts', joint3_ts)
                    case 7  
                        joint3_ts.Data = temp;
                        temp =  joint1_ts.Data;                    
                        joint1_ts.Data = extend_trajectory(joint1_ts.Data, scale_factor);
                        assignin('base','joint1_ts', joint1_ts)
                    case 8
                        joint1_ts.Data = temp;
                        temp = joint2_ts.Data;
                        joint2_ts.Data = extend_trajectory(joint2_ts.Data, scale_factor);
                        assignin('base','joint2_ts', joint2_ts)
                    case 9
                        joint2_ts.Data = temp;
                        temp = joint3_ts.Data;
                        joint3_ts.Data = extend_trajectory(joint3_ts.Data, scale_factor);
                        assignin('base','joint3_ts', joint3_ts)
                    case 10
                        joint3_ts.Data = temp;
                        temp = joint1_ts.Data;                    
                        joint1_ts.Data = process_points_capped_speed(joint1_ts.Data, speedcap, timescale);
                        assignin('base','joint1_ts', joint1_ts)
                    case 11
                        joint1_ts.Data = temp;
                        temp = joint2_ts.Data;                    
                        joint2_ts.Data = process_points_capped_speed(joint2_ts.Data, speedcap, timescale);
                        assignin('base','joint2_ts', joint2_ts)
                    case 12
                        joint2_ts.Data = temp;
                        temp = joint3_ts.Data;                    
                        joint3_ts.Data = process_points_capped_speed(joint3_ts.Data, speedcap, timescale);
                        assignin('base','joint3_ts', joint3_ts)
                    case 13
                        error4=m0;
                        assignin('base','error4', error4)
                    case 14
                        error5=m0;
                        assignin('base','error4', error4)
                    case 15
                        error6=m0;
                        assignin('base','error4', error4)
                end
                dataset2 = [dataset2, targets];
        
                %on ajoute déjà les trajectoires cibles
                disp("----------------")
                disp("----------------")
                simOut = sim(model_name);
                assignin('base','simOut', simOut)
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
        
            
                [x, y, z] = ForwardKinematic(j1o, j2o, j3o, j4o, j5o,len_time_series); 
                jdatapoint = [x, y, z];%pour un j donné on met à la suite les len_time_series prédit  et les réels en prenant en compte le défault moteur, c'est ce qu'on donnera à manger à l'IA;

                
                dataset2=[dataset2,jdatapoint];
        
                
                
            end

        end
        fprintf("The final size of the dataset2 is %s", mat2str(size(dataset2)));
        v2=mat2str(size(dataset2));
        dataset = [dataset, dataset2];
    end
    % Specify the size of each submatrix (6x1000)
    fprintf("The final size of the fulldataset is %s", mat2str(size(dataset)));
    v3=mat2str(size(dataset));
    dataset = dataset';
    %clear size
    sized = size(dataset);

    rowDist = 6 * ones(1, sized(1)/6);
    % Use mat2cell to convert the dataset into a cell array
    cellArray = mat2cell(dataset, rowDist);
    disp(size(cellArray))
    
    save(trajectory_dataset_name, 'cellArray');
    % Now, cellArray is a cell array where each cell is a 6x1000 matrix

    %Running the rain_predict_file
    %run('rain_predict_lstm.m');
    %run("optimum_train_predict.m")

    %%% end of experimental section %%%


end











function [x, y ,z] = ForwardKinematic(j1, j2, j3, j4, j5,len_time_series)
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
    T = 10; % period
    %spline = zeros(len_time_series,5);
    
    len = size(j1);
    for i = 1:len_time_series
        targets = [j1(i),j2(i),j3(i),j4(i),j5(i)];
    

    
        [outputVec,statusFlag] = solve(ik,targets);
        x(i,1) = outputVec(1);
        y(i,1) = outputVec(2);
        z(i,1) = outputVec(3);
    
        
    end
    



end

function updated_j1 = process_points(pointsList, j1)
    % Check if input lists have the same length
    if numel(pointsList) ~= numel(j1)
        error('Input lists must have the same length.');
    end

    % Iterate over pointsList
    for i = 2:numel(pointsList)
        if pointsList(i) == 0
            % Set j1(i) to j1(i-1)
            
            j1(i) = j1(i-1);
            
            % Update all j1 values with indices greater than i
            j1(i+1:end) = j1(i:end-1);
        end
    end

    % Return the updated j1 list
    updated_j1 = j1;
end

function updated_j1 = process_points_capped_speed(j1, cap, time_scale)
%sample values : cap = 2, time_scale  = 0.01
    for i = 2:numel(j1)
          
        if (j1(i)-j1(i-1))/time_scale > cap
            % Set j1(i) to j1(i-1)
            
            j1(i) = j1(i-1)+cap*time_scale;
        elseif (j1(i)-j1(i-1))/time_scale < - cap

            j1(i) = j1(i-1)-cap*time_scale;
            
        end
    end

    % Return the updated j1 list
    updated_j1 = j1;
end





function updated_trajectory = extend_trajectory(originalPoints, scaleFactor)
    % Determine the number of points in the original trajectory
    numOriginalPoints = size(originalPoints, 1);

    % Create an index for the original points
    originalIndices = 1:numOriginalPoints;

    % Create an extended index based on the scale factor
    extendedIndices = linspace(1, numOriginalPoints, round(scaleFactor * numOriginalPoints));

    % Interpolate to get extended trajectory
    
    updated_trajectory = interp1(originalIndices, originalPoints, extendedIndices, 'linear', 'extrap');

    % Limit the extended trajectory to the original number of points
    updated_trajectory = updated_trajectory(1:min(numOriginalPoints, length(extendedIndices)));
end

%function0
function reducedStruct = reduceStructureSize(inputStruct, n)
    % Get field names of the input structure
    fieldNames = fieldnames(inputStruct);
    
    % Check if n is greater than the number of fields
    if n > numel(fieldNames)
        error('n is greater than the number of fields in the input structure.');
    end
    
    % Randomly select n field names
    selectedFields = datasample(fieldNames, n, 'Replace', false);
    
    % Create the reduced structure
    reducedStruct = struct();
    for i = 1:numel(selectedFields)
        fieldName = selectedFields{i};
        reducedStruct.(fieldName) = inputStruct.(fieldName);
    end
end

%function1
function R = calculateRotationMatrix(anglexy, anglexz, angleyz)
    % Conversion des angles en radians
    anglexy = deg2rad(anglexy);
    anglexz = deg2rad(anglexz);
    angleyz = deg2rad(angleyz);

    % Matrices de rotation autour des axes
    Rz = [cos(anglexy) -sin(anglexy) 0; sin(anglexy) cos(anglexy) 0; 0 0 1];
    Rx = [1 0 0; 0 cos(anglexz) -sin(anglexz); 0 sin(anglexz) cos(anglexz)];
    Ry = [cos(angleyz) 0 sin(angleyz); 0 1 0; -sin(angleyz) 0 cos(angleyz)];

    % Calcul de la matrice totale de rotation
    R = Rz * Rx * Ry;
end


%function2
function newCell = multiplyandsum(r,matrix1,Cell,matrix2)

newCell = cell(size(Cell));
   for i=1:numel(Cell)
      

       scaledfunction= @(x) r*(matrix1(i, 1) *Cell{1}(x) + matrix1(i, 2) * Cell{2}(x) + matrix1(i, 3) * Cell{3}(x))+matrix2(i);
       newCell{i}= scaledfunction;
   end
end

%function3
function structure3 = mergeStructures(structure1, structure2)
    % Copy the contents of structure1 to structure3
    structure3 = structure1;
    
    % Get field names of structure2
    fields2 = fieldnames(structure2);

    % Iterate through fields of structure2 and add them to structure3
    for i = 1:length(fields2)
        field = fields2{i};
        structure3.(field) = structure2.(field);
    end
end



%function4
function [circlelist] = CreateCircleList(max_rayon,max_eloignement_centre)
        %il va falloir limitter les paramètres maximum de la génération de
        %form en fonction du mouvement permi par le bras
        %pas de 0.01 choisi dans premières boucles for experimentalement
        %valeures de min et max eloignement aussi
        %Valeures charac du robot
        min_eloignement=0.02;
        max_eloignement=0.28;
        circlelist=struct();
        x_prime_z_prime_y_prime_coords={@(t) cos(2*pi*t);@(t) sin(2*pi*t);@(t) 0};
        for e_h=10:1:max_eloignement_centre*100                      %on itère sur les rayons possibles#changer incrémentation
            e=e_h*0.01;
            for r_h =1:1:max_rayon*100                             %on itère sur l'éloignement au centre possible #changer incrémentation?
                r=r_h*0.01;
                %condition d'appliquabilité à la simulation,trop forte
                %vire des points qui marche mais peu de calculs
                if (abs(e-r)<min_eloignement) || (e+r<min_eloignement) || (e+r>max_eloignement) || (abs(e-r)>max_eloignement)
                    continue
                end
                for anglexy=0:120:360                         %on explore les plans possibles en effectuant des rotations du plan xy autour de z
                    for anglexz=0:120:360                     %on explore les plans possibles en effectuant des rotations du plan xz autour de y
                        for angleyz=0:120:360                 %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
                            for anglez=0:120:360              %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
                                for anglex=0:120:360          %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
                                    for angley=0:120:360      %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
                                        %coordonnées du centre du cercle
                                        %tracé
                                        ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
                                        Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
                                        circlecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
                                        thiscircle=struct();
                                        circlename = sprintf('c_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
                                        fieldName = sprintf('xequation');
                                        thiscircle.(fieldName)=circlecoords{1};
                                        fieldName = sprintf('yequation');
                                        thiscircle.(fieldName)=circlecoords{2};
                                        fieldName = sprintf('zequation');
                                        thiscircle.(fieldName)=circlecoords{3};
                                        circlelist.(circlename) = thiscircle;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end



%function5
function [linelist] = CreateLineList(max_eloignement_centre,max_longueur)
        %il va falloir limitter les paramètres maximum de la génération de
        %form en fonction du mouvement permi par le bras
        linelist=struct();
        min_eloignement=0.02;
        max_eloignement=0.28;
        x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)*t;@(t) 0;@(t) 0};
        for e_h=10:1:max_eloignement_centre*100         %on itère sur l'éloignement au centre possible #changer incrémentation?
            e=e_h*0.01;
            for r_h =1:1:max_longueur*100
                r=r_h*0.01;
                %condition d'appliquabilité à la simulation, trop forte
                %vire des points qui marche mais peu de calculs
                if (abs(e-r)<min_eloignement) || (e+r<min_eloignement) || (e+r>max_eloignement) || (abs(e-r)>max_eloignement)
                    continue
                end
                for anglexy=0:120:360                       %on explore les plans possibles en effectuant des rotations du plan xy autour de z
                    for anglexz=0:120:360                   %on explore les plans possibles en effectuant des rotations du plan xz autour de y
                        for angleyz=0:120:0%fait rien ici 0 %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
                            for anglez=0:120:360            %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
                                for anglex=0:120:360        %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
                                    for angley=0:120:360    %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
                                        %coordonnées du centre du cercle
                                        %tracé
                                        ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
                                        Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
                                        linecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
                                        thisline=struct();
                                        linename = sprintf('l_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
                                        fieldName = sprintf('xequation');
                                        thisline.(fieldName)=linecoords{1};
                                        fieldName = sprintf('yequation');
                                        thisline.(fieldName)=linecoords{2};
                                        fieldName = sprintf('zequation');
                                        thisline.(fieldName)=linecoords{3};
                                        linelist.(linename) = thisline;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
end


%function6
% function [rectanglelist] = Create_rectangle_List(max_eloignement_centre,max_longueur,max_largeur)
%         %il va falloir limitter les paramètres maximum de la génération de
%         %form en fonction du mouvement permi par le bras
%         rectanglelist=struct();
%         r=1;
%         x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)t-(max_longueur+max_largeur<t<=2*max_longueur+max_largeur)(t-max_longueur+max_largeur);@(t) (max_longueur<t<=max_longueur+max_largeur)(t-max_longueur) - (2*max_longueur+max_largeur<t<=2*max_longueur+2*max_largeur)(t-2*max_longueur+max_largeur);@(t) 0};
%         for e=0:max_eloignement_centre                  %on itère sur l'éloignement au centre possible #changer incrémentation?
%             for anglexy=0:120:360                       %on explore les plans possibles en effectuant des rotations du plan xy autour de z
%                 for anglexz=0:120:360                   %on explore les plans possibles en effectuant des rotations du plan xz autour de y
%                     for angleyz=0:120:360               %on explore les plans possibles en effectuant des rotations du plan yz autour de x          
%                         for anglez=0:120:360            %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan z
%                             for anglex=0:120:360        %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan x
%                                 for angley=0:120:360    %on génère par incrément de 1 degré un "cercle" formé des centre des cercles que l'on va tracer à la distance voulue sur le plan y
%                                     %coordonnées du centre du cercle
%                                     %tracé
%                                     ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
%                                     Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
%                                     rectanglecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);
%                                     thisrectangle=struct();
%                                     rectanglename = sprintf('r_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r, e, anglexy, anglexz, angleyz, anglez, anglex, angley);
%                                     fieldName = sprintf('xequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{1};
%                                     fieldName = sprintf('yequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{2};
%                                     fieldName = sprintf('zequation');
%                                     thisrectangle.(fieldName)=rectanglecoords{3};
%                                     rectanglelist.(rectanglename) = thisrectangle;
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
%         end
% end

%function7
function [interpolated_set] = createInterpolate(numberofinterpolatedshapes,len_time_series)
    %Interpolation set creation
    interpolated_set = struct();
    min_eloignement_point=0.02;
    max_eloignement_point=0.28;
    
    for p = 1:numberofinterpolatedshapes
        thisshape=struct();
        num_point = randi([3, 10]); % number of point for interpolation
        m = (max_eloignement_point - min_eloignement_point) * rand(3, num_point) + min_eloignement_point;
    
        %verification of sufficient Z value
        for i = 1:num_point
            if m(3,i)<0.1
                m(3,i)=0.1+(max_eloignement_point-0.1)*m(3,i);
            end
        end
    
        shapename = sprintf('ishape_p%d_num_point%d', p, num_point);
        X = m(1,:);Y = m(2,:);Z = m(3,:);
        values = spcrv([X(1) X X(end);Y(1) Y Y(end);Z(1) Z Z(end)],4);
        % plot3(X,Y,Z)
        % 
        % plot3(values(1,:),values(2,:),values(3,:))
        
        ts_x = timeseries(values(1,:),linspace(0,10,size(values,2)));
        ts_y = timeseries(values(2,:),linspace(0,10,size(values,2)));
        ts_z = timeseries(values(3,:),linspace(0,10,size(values,2)));
        
        end_time_value_in_seconds= (len_time_series-1)*0.01;

        ts_x = resample(ts_x, 0:0.01:end_time_value_in_seconds);
        ts_y = resample(ts_y, 0:0.01:end_time_value_in_seconds);
        ts_z = resample(ts_z, 0:0.01:end_time_value_in_seconds);

        % ts_x = resample(ts_x, 0.01:0.01:10);
        % ts_y = resample(ts_y, 0.01:0.01:10);
        % ts_z = resample(ts_z, 0.01:0.01:10);
        % 
        
        x = ts_x.Data(:);
        y = ts_y.Data(:);
        z = ts_z.Data(:);
        fieldName = sprintf('xcoords');
        thisshape.(fieldName)=x;
        fieldName = sprintf('ycoords');
        thisshape.(fieldName)=y;
        fieldName = sprintf('zcoords');
        thisshape.(fieldName)=z;
        interpolated_set.(shapename) = thisshape;
    end
end

%function8
function [circlelist] = CreateRandomCircleList(max_rayon, num_trajectories)
    max_eloignement_centre=max_rayon;
    min_eloignement = 0.02;
    max_eloignement = 0.28;
    min_hauteur = 0.1;
    circlelist = struct();
    x_prime_z_prime_y_prime_coords = {@(t) cos(2*pi*t); @(t) sin(2*pi*t); @(t) 0};

    % Initialize counter
    generated_trajectories = 0;

    % Keep generating trajectories until the desired number is reached
    while generated_trajectories < num_trajectories
        % Choose indices randomly
        e_values = generateRandomNumbers(min_hauteur*100, max_eloignement_centre*100, 1);
        r_values = generateRandomNumbers(1, max_rayon*100, 1);
        anglexy_values = generateRandomNumbers(1, 360, 1);
        anglexz_values = generateRandomNumbers(1, 360, 1);
        angleyz_values = generateRandomNumbers(1, 360, 1);
        anglez_values = generateRandomNumbers(1, 360, 1);
        anglex_values = generateRandomNumbers(1, 360, 1);
        angley_values = generateRandomNumbers(1, 360, 1);

        % Create a single trajectory
        for i = 1:1
            % Choosing values
            e_h = e_values(i);
            r_h = r_values(i);
            e = e_h * 0.01;
            r = r_h * 0.01;

            % Filtering values that would cause clipping
            if (abs(e-r) < min_eloignement) || (e+r < min_eloignement) || (e+r > max_eloignement) || (abs(e-r) > max_eloignement)
                continue
            end

            anglexy = anglexy_values(i);
            anglexz = anglexz_values(i);
            angleyz = angleyz_values(i);
            anglez = anglez_values(i);
            anglex = anglex_values(i);
            angley = angley_values(i);

            % Writing the functions
            ecoord = e * calculateRotationMatrix(anglez, angley, anglex) * [1; 1; 1];
            Rotation = calculateRotationMatrix(anglexy, anglexz, angleyz);
            circlecoords = multiplyandsum(r, Rotation, x_prime_z_prime_y_prime_coords, ecoord);

            % Create a structure for this trajectory
            thiscircle = struct();
            circlename = sprintf('c_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
            fieldName = sprintf('xequation');
            thiscircle.(fieldName) = circlecoords{1};
            fieldName = sprintf('yequation');
            thiscircle.(fieldName) = circlecoords{2};
            fieldName = sprintf('zequation');
            thiscircle.(fieldName) = circlecoords{3};

            % Add this trajectory to the structure
            circlelist.(circlename) = thiscircle;

            % Increment the counter
            generated_trajectories = generated_trajectories + 1;
        end
    end
end

%function9
function [linelist] = CreateRandomLineList(max_longueur, num_trajectories)
    max_eloignement_centre =max_longueur;
    min_eloignement = 0.02;
    max_eloignement = 0.28;
    min_hauteur = 0.1;
    linelist=struct();
    x_prime_z_prime_y_prime_coords={@(t) (t<=max_longueur)*t;@(t) 0;@(t) 0};
    % Initialize counter
    generated_trajectories = 0;

    % Keep generating trajectories until the desired number is reached
    while generated_trajectories < num_trajectories
        % Choose indices randomly
        e_values = generateRandomNumbers(min_hauteur*100, max_eloignement_centre*100, 1);
        r_values = generateRandomNumbers(1, max_longueur*100, 1);
        anglexy_values = generateRandomNumbers(1, 360, 1);
        anglexz_values = generateRandomNumbers(1, 360, 1);
        angleyz_values = generateRandomNumbers(1, 360, 1);
        anglez_values = generateRandomNumbers(1, 360, 1);
        anglex_values = generateRandomNumbers(1, 360, 1);
        angley_values = generateRandomNumbers(1, 360, 1);

        % Create a single trajectory
        for i = 1:1
            % Choosing values
            e_h = e_values(i);
            r_h = r_values(i);
            e = e_h * 0.01;
            r = r_h * 0.01;

            % Filtering values that would cause clipping
            if (abs(e-r) < min_eloignement) || (e+r < min_eloignement) || (e+r > max_eloignement) || (abs(e-r) > max_eloignement)
                continue
            end

            anglexy = anglexy_values(i);
            anglexz = anglexz_values(i);
            angleyz = angleyz_values(i);
            anglez = anglez_values(i);
            anglex = anglex_values(i);
            angley = angley_values(i);

            % Writing the functions
            ecoord=e*calculateRotationMatrix(anglez, angley, anglex)*[1;1;1];
            Rotation=calculateRotationMatrix(anglexy, anglexz, angleyz);
            linecoords=multiplyandsum(r,Rotation,x_prime_z_prime_y_prime_coords,ecoord);

            % Create a structure for this trajectory
            thisline=struct();
            linename = sprintf('l_r%d_e%d_xy%d_xz%d_yz%d_z%d_x%d_y%d', r_h, e_h, anglexy, anglexz, angleyz, anglez, anglex, angley);
            fieldName = sprintf('xequation');
            thisline.(fieldName)=linecoords{1};
            fieldName = sprintf('yequation');
            thisline.(fieldName)=linecoords{2};
            fieldName = sprintf('zequation');
            thisline.(fieldName)=linecoords{3};

            % Add this trajectory to the structure
            linelist.(linename) = thisline;

            % Increment the counter
            generated_trajectories = generated_trajectories + 1;
        end
    end
end

%function10
function randomIntegers = generateRandomNumbers(a, b, p)
    % Generate p random integers in the interval [a, b]
    randomIntegers = randi([round(a), round(b)], 1, p);
end