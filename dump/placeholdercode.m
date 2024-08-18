%% calling the functions to generate data

%the following line creates a sample item. testdataset contains a cell,
%with numberTrajectories cells inside -each one represents a trajectory,
%and contains 5 matrices containing the data over time, with numberPoints
%of points, sampled at 0.1s intervals.
%percentageNoCommand serves to fix a certain percentage of motors as not being under command
%(otherwise the code will always give a non-constant trajectory to each
%point). Can be set freely between 0 (all motors under command) and 1 (no motors under command).

%testcsv serves to rebuild a trajectory for manual movement.


%NB : data provided in degrees. To switch to the control units of the
%robot, divide by 0.24. 


%NB2 : some of the lower-level parameters (max speed, number of inflexions
%per trajectory) are set on a function level; see function definitions for
%details, esp. "realisticsinglemotorcommand". Feel free to ask for further
%details, or a more accesibly paramterised model.


numberTrajectories = 3 ;
numberPoints = 1000;
percentageNoCommand = 0;


[testdataset,testcsv] = createRandomPickupList(numberTrajectories,numberPoints, percentageNoCommand);


%% the following line serves to turn the Matlab timeseries (with regular timestamps) into a .csv. I believe the output .csv of this
%file is the one you are interested in
create_csv_timeseries(testdataset)

%% the following line serves to turn the previously generated pointwise Matlab cells
%into .csv files; these files are directly created in the workspace. 

create_csv(testdataset,testcsv);



%% display

%the following section serves no direct function, and may be commented or
%removed - it simply displays the motor-by-motor curves generated for the
%first trajectory, in order to visualise results.

% a = testdataset(1);
% b = a{1};
% for j = 1:5
%     c = b{j};
%     figure;
%     plot(c)
% end


%% - implementation of the functions

function [trajectories,csv_file_equivalent] = createRandomPickupList(number_of_pickup_trajctories,len_time_series, zero_amount,average_smallest_motive_lenght)
%_____
%Creates a structure containing a wanted number of  random trajectories that mimic
%a realistic pickup mouvement of a wanted length ( 5 commands, 1 for each
%motor)
%
%   number_of_pickup_trajctories: number of trajectories that are given
%   back
%   len_time_series: number of points intrajectories that are given back
%_____

%Setting  default value to average_smallest_motive_lenght
 if nargin < 4
        average_smallest_motive_lenght=50; % Set a default value for average_smallest_motive_lenght
 end
% Check if the condition is not met
if len_time_series<average_smallest_motive_lenght
    error('Condition not met: len_time_series<average_smallest_motive_lenght, either reduce number of points for minimal motive or make the trajectory longer'); % Raise an error
end
%Interpolation set creation
    pickup_set = struct();
    min_eloignement_point=0.02;
    max_eloignement_point=0.28;
    
    max_number_of_motives=len_time_series/average_smallest_motive_lenght;

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
        for i = 1:5
            [motor_command,triplets] = realisticsinglemotorcommand(max_number_of_motives,len_time_series,zero_amount);
            trajectory{i} = motor_command;
            triplets_cell{i} = triplets;
        end

        trajectories{generated_trajectories} = trajectory;
        csv_file_equivalent{generated_trajectories} = triplets_cell;
       
    end
end

function [motor_command, triplets] = realisticsinglemotorcommand(max_number_of_motives,len_time_series,zero_amount)
%_____
%Returns the keypoints that are representative of a pickup mouvement, these
%can be used to make the real robot perform the trajectory. It is
%considered that the robot is initially in a random position, will have to
%move to a position to pick up an object and then move that object to a
%final position.
%
%   n: case that is executed 
%_____

%Random number of motives on the command of each motor
%Note to self: maybe favorise apparition of 0 more often with better
%mechanism?

%Initialising the points at 0
motor_command = zeros(len_time_series,1);


%Choice of motor use or not
percentage_zero_amount= zero_amount*100;
toggle_value= generateRandomNumbers(1,100, 1);
average_smallest_motive_lenght=len_time_series/max_number_of_motives;

%Point generation range values
min_command=0;
max_command=360;
speed_cap=2.7;

%Application of the toggle
if toggle_value>=percentage_zero_amount
    
    number_of_motives_in_traj =5 ; %NB : In order to change the number of points, we reccomend changing the division factor here
    triplets=zeros(number_of_motives_in_traj,3);

    %needs testing
    remaining_points=len_time_series;
    current_index=0;
    old_point=0;
    for i =1:number_of_motives_in_traj
        %The two durations (one time top arrive to the point associated with the
        %motive and one for the plateau after attaining this point)
    
        %inside motif arrival to plateau len rapport
        arrival_to_plateau_proportion=generateRandomNumbers(1,10,1)/10;
    
        %motive len
        motive_len=200;
    
        %arrival to point len
        arrival_to_point_lenght=round(motive_len*arrival_to_plateau_proportion);
        triplets(i,2)=arrival_to_point_lenght;
        if i ~= 1
            old_point=motor_command(current_index,1);
        end 
        
    
        %plateau len


        plateau_lenght=motive_len-arrival_to_point_lenght;
        triplets(i,3)=plateau_lenght;

      
        
        point=generateRandomNumbers(max(0,old_point-speed_cap* arrival_to_point_lenght),min(360,old_point+speed_cap* arrival_to_point_lenght),1);
        triplets(i,1)=point;
            
        %motor_comamand_updates
        motor_command(current_index+1:current_index+arrival_to_point_lenght,1)=linspace(old_point, point,arrival_to_point_lenght);
        motor_command(current_index+arrival_to_point_lenght+1:current_index+motive_len,1)=point;

        %calculation of remaining points to be used 
        remaining_points=remaining_points-motive_len;
        current_index=len_time_series-remaining_points;
        

    end
else
    triplets=[0,0,0];
end

end

%function10
function randomIntegers = generateRandomNumbers(a, b, p)
    % Generate p random integers in the interval [a, b]
    randomIntegers = randi([round(a), round(b)], 1, p);
end


%function 11 - this function serves to create timestamp arrays for csv
%conversion
function new_array = create_timestamped_array(original_array)
    n = size(original_array, 1);
    new_array = zeros(n, 2);
    
    % Copy the first column unchanged
    new_array(:, 1) = original_array(:, 1);
    
    % Initialize a variable to store the cumulative sum
    cumulative_sum = 0;
    
    % Calculate the second column for the new array
    for l = 1:n
        % Add the sum of second and third columns of previous lines
        cumulative_sum = cumulative_sum + original_array(l, 2) + original_array(l, 3);
        
        % Calculate the new second column
        new_array(l, 2) = cumulative_sum;
    end
end



function modified_array = altered_modify_array(B, values)
    % Sort the list of values in ascending order
    sorted_values = sort(values);
    
    % Initialize an empty array to store the modified array
    modified_array = [];

    % Iterate through the sorted list of values
    for i = 1:length(sorted_values)
        current_value = sorted_values(i);
        
        % Check that the current value is within the range of indices in B
        if current_value >= 1 && current_value <= size(B, 1)
            % Get the corresponding row from B using the floor of the current value
            new_row = [B(floor(current_value), 1), current_value];
            
            % Append the new row to the modified array
            modified_array = [modified_array; new_row];
        end
    end
end



%function 13
%this function serves to generate .csv for conversion to the real system -
%note then inversion of motor order to match the real robot's and the
%addition of a duplicate for the pincer command we do not provide
function save_to_csv(O1, O2, O3, O4, O5, filename)
    disp(O1)
    n = size(O1, 1);
    disp(n)
    final_matrix = zeros(n, 7);
    disp(size(final_matrix(:, 1)))
    disp(O5(:, 1))
     disp(O4(:, 1))
      disp(O3(:, 1))
       disp(size(O2(:, 1)))
        disp(size(O1(:, 1)))
    final_matrix(:, 1) = O1(:, 2); 
    final_matrix(:, 2) = O5(:, 1);
    final_matrix(:, 3) = O5(:, 1);
    final_matrix(:, 4) = O4(:, 1);
    final_matrix(:, 5) = O3(:, 1);
    final_matrix(:, 6) = O2(:, 1);
    final_matrix(:, 7) = O1(:, 1);
    headers = {'timestamp', 'motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6'};
    data_table = array2table(final_matrix, 'VariableNames', headers);
    writetable(data_table, filename);
    disp(['CSV file saved as: ', filename]);
end

%function 14 - this function serves to create a tiemstamped list of
%durations rather than points
function new_array = duration_timestamped_array(original_array)
    n = size(original_array, 1);
    new_array = zeros(n, 2);
    new_array(:, 1) = original_array(:, 3);
    cumulative_sum = 0;
    for l = 1:n
        cumulative_sum = cumulative_sum + original_array(l, 2) + original_array(l, 3);
        new_array(l, 2) = cumulative_sum;
    end
end

%function 16 - this function serves to generate a duration csv ready for
%use
function tempArray = modifyDurationSet(pointSet, durationSet)
    for k = 1:length(pointSet)
        tempArray = pointSet;
        tempArray(:, 1) = 0;  

        % Step b: Process durationSet{k}F
        durationArray = durationSet;
        for i = 1:size(durationArray, 1)
            point = durationArray(i, 1);
            timestamp = floor(durationArray(i, 2));
            index = find(tempArray(:, 2) == timestamp, 1);
            
            % If the index is found
            if ~isempty(index)
                tempValue = point;
                
                % Keep processing as long as there is a value to assign
                while tempValue > 0 && index <= size(tempArray, 1)    
                    currentTimestamp = tempArray(index, 2);
                    if index < size(tempArray, 1)
                        nextTimestamp = tempArray(index + 1, 2);
                    else
                        nextTimestamp = inf;  
                    end

                    timeDifference = nextTimestamp - currentTimestamp;

                    %the following if loop serves to "spread out" the
                    %duration in the event that it is more important than
                    %the time before the next point - it passes kon the
                    %exceeding time with said next point, and so on.
                    if tempValue >= timeDifference
                        tempArray(index, 1) = timeDifference;
                        tempValue = tempValue - timeDifference;
                        index = index + 1;
                    else
                        tempArray(index, 1) = tempValue;
                        tempValue = 0;
                    end
                end
            end
        end


    end
end


%function 16 - this function serves to turn the outputs of  = createRandomPickupList
function create_csv(testdataset,testcsv)
for i = 1:length(testcsv)
    pointSet = testcsv{i};
    durationSet = testcsv{i};
    datapoints = testdataset{i};
    
    for j = 1:5
        pointSet{j} = create_timestamped_array(pointSet{j});
        durationSet{j} = duration_timestamped_array(durationSet{j});

      
    end
        secondColumns = cellfun(@(x) x(:, 2), pointSet, 'UniformOutput', false);
        combinedArray = vertcat(secondColumns{:});
        
        uniqueValues = unique(combinedArray);
        uniqueValues = uniqueValues(uniqueValues ~= 0 & uniqueValues ~= 1000);
        uniqueValues = sort(uniqueValues);


        for k = 1:5
       
        pointSet{k} = altered_modify_array(datapoints{k}, uniqueValues);
        durationSet{k} = modifyDurationSet(pointSet{k},durationSet{k});
    end

   

     save_to_csv(pointSet{1}, pointSet{2}, pointSet{3}, pointSet{4}, pointSet{5}, "trajectory_points"+ string(i)+".csv")
     disp(' trajectory csv created succesfully')
     save_to_csv(durationSet{1}, durationSet{2}, durationSet{3}, durationSet{4}, durationSet{5}, "duration_points"+ string(i)+".csv")
     disp('duration csv created succesfully')
end

end

%function 17  -creates a csv for the full timeseries, rather than the
%points
function create_csv_timeseries(testdataset)
for i = 1:length(testdataset)
    datapoints = testdataset{i};
    disp(datapoints)
    disp("##############")
    for j = 1:5
        pointSet{j} = datapoints{j};
        disp(pointSet)
    end

     save_to_csv_timeseries(pointSet{1}, pointSet{2}, pointSet{3}, pointSet{4}, pointSet{5}, "trajectory_monitoring_position"+ string(i)+".csv")
     disp(' trajectory csv created succesfully')
end


end
% function 18 -same concept as the previous save to .csv, adapted to use
% the timeseries directly rather than the points
function save_to_csv_timeseries(O1, O2, O3, O4, O5, filename)
    n = size(O1, 1);
    disp(n)
    final_matrix = zeros(n, 7);
    disp(final_matrix(:, 1))
    final_matrix(:, 1) = linspace(0,  0.1,n); 
    final_matrix(:, 2) = O5(:, 1);
    final_matrix(:, 3) = O5(:, 1);
    final_matrix(:, 4) = O4(:, 1);
    final_matrix(:, 5) = O3(:, 1);
    final_matrix(:, 6) = O2(:, 1);
    final_matrix(:, 7) = O1(:, 1);
    headers = {'timestamp', 'motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6'};
    data_table = array2table(final_matrix, 'VariableNames', headers);
    writetable(data_table, filename);
    disp(['CSV file saved as: ', filename]);
end