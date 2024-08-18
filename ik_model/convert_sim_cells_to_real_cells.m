% Number of trajectories (subfolders)
numTrajectories = numel(realistic_trajs_representative_point_set);

% Get the current time in seconds since UNIX epoch, including decimal precision
currentTime = posixtime(datetime('now'));

% Timestamp for the CSVs with 5 rows (increments by 2 seconds from current time)
timestamp_5 = currentTime + (0:2:8)';

% Timestamp for the CSV with 82 rows (evenly spaced increments from current time over 8 seconds)
timestamp_82 = linspace(currentTime, currentTime + 8, 82)';

% Column titles for the CSVs
columnTitles = {'timestamp', 'motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6'};

% Root folder name
rootFolder = 'C:\Users\PC\Downloads\data_for_training\created_from_sim_matrices';

% Create root folder if it doesn't exist
if ~exist(rootFolder, 'dir')
    mkdir(rootFolder);
end

% Iterate through each trajectory
for trajIdx = 1:numTrajectories
    % Create subfolder for each trajectory
    subfolderName = fullfile(rootFolder, ['trajectory_' num2str(trajIdx)]);
    if ~exist(subfolderName, 'dir')
        mkdir(subfolderName);
    end
    
    % Extract data for current trajectory
    trajectoryData = realistic_trajs_representative_point_set{trajIdx};
    
    % Initialize matrices for the 3 CSV files
    monitoring_cmd = zeros(5, 7); % 5 rows, 7 columns (timestamp + 6 motors)
    monitoring_cmd_duration = zeros(5, 7);
    monitoring_position = zeros(82, 7); % 82 rows, 7 columns
    
    % Populate the matrices
    for motorIdx = 1:5
        % Motor data (5x3 double matrix)
        motorData = trajectoryData{motorIdx};
        
        % Map to motors in reverse order
        reverseMotorIdx = 7 - motorIdx; % motorIdx:1 -> reverseMotorIdx:6, etc.
        
        % Fill the monitoring_cmd matrix (1st column of each motor), rescale and add 500
        monitoring_cmd(:, reverseMotorIdx) = floor(motorData(:, 1) / 0.24 + 500);
        
        % Fill the monitoring_cmd_duration matrix (2nd column of each motor)
        monitoring_cmd_duration(:, reverseMotorIdx) = motorData(:, 2);
    end
    
    % Duplicate motor 2's data for motor 1 in the monitoring_cmd and monitoring_cmd_duration matrices
    monitoring_cmd(:, 7) = monitoring_cmd(:, 6);  % Copy motor 2's data to motor 1
    monitoring_cmd_duration(:, 7) = monitoring_cmd_duration(:, 6);  % Copy motor 2's data to motor 1
    
    % Add the timestamp column to the monitoring_cmd and monitoring_cmd_duration matrices
    monitoring_cmd(:, 1) = timestamp_5;
    monitoring_cmd_duration(:, 1) = timestamp_5;
    
    % For the monitoring_position matrix, fill the timestamp and random data
    monitoring_position(:, 1) = timestamp_82;
    monitoring_position(:, 2:end) = rand(82, 6);  % Random data for motor columns
    
    % Write the CSV files in the respective subfolder with column titles
    write_csv_with_header(fullfile(subfolderName, 'trajectory_monitoring_cmd.csv'), monitoring_cmd, columnTitles);
    write_csv_with_header(fullfile(subfolderName, 'trajectory_monitoring_cmd_duration.csv'), monitoring_cmd_duration, columnTitles);
    write_csv_with_header(fullfile(subfolderName, 'trajectory_monitoring_position.csv'), monitoring_position, columnTitles);
end

disp('CSV files have been created successfully.');

% Function to write CSV with header and precise timestamps
function write_csv_with_header(filename, data, headers)
    % Convert headers to comma-separated string
    headerStr = strjoin(headers, ',');
    
    % Open the file for writing
    fid = fopen(filename, 'w');
    
    % Write the header
    fprintf(fid, '%s\n', headerStr);
    
    % Write the data row by row to keep precision for timestamps
    for row = 1:size(data, 1)
        % Write each row with full precision for all numbers
        fprintf(fid, '%.10f,', data(row, 1));  % Full precision for timestamp
        fprintf(fid, '%.10f,', data(row, 2:end-1));  % Full precision for motors
        fprintf(fid, '%.10f\n', data(row, end));  % Last motor without trailing comma
    end
    
    % Close the file
    fclose(fid);
end
