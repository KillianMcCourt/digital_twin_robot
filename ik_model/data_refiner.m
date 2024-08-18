% Define the directory containing split data
baseDir = 'C:\Users\PC\Downloads\data_for_training\data_for_training';

% Get a list of all subfolders in the base directory
subfolders = dir(baseDir);
subfolders = subfolders([subfolders.isdir]); % Filter only directories

% Iterate over each subfolder
for k = 1:length(subfolders)
    folderName = subfolders(k).name;
    if startsWith(folderName, '.') % Skip hidden directories like '.' and '..'
        continue;
    end
    
    subfolderPath = fullfile(baseDir, folderName);
    
    % Load command and duration CSV files
    cmdFile = fullfile(subfolderPath, 'trajectory_monitoring_cmd.csv');
    cmdDurationFile = fullfile(subfolderPath, 'trajectory_monitoring_cmd_duration.csv');
    positionFile = fullfile(subfolderPath, 'trajectory_monitoring_position.csv');
    
    cmdData = readtable(cmdFile);
    cmdDurationData = readtable(cmdDurationFile);
    positionData = readtable(positionFile);
    
    % Remove the first and last lines from command and duration matrices
    if height(cmdData) > 2
        cmdData = cmdData(2:end-1, :);
        cmdDurationData = cmdDurationData(2:end-1, :);
    else
        fprintf('Skipping folder %s: Not enough data to remove first and last lines.\n', folderName);
        continue;
    end
    
    % Determine new time range for position data
    newEarliestTimestamp = min(cmdData.timestamp) + 1;
    newLatestTimestamp = max(cmdData.timestamp) + 1;
    
    % Filter position data within the new time range
    positionData = positionData(positionData.timestamp >= newEarliestTimestamp & positionData.timestamp <= newLatestTimestamp, :);
    
    % Save the modified command, duration, and position data back to files
    writetable(cmdData, cmdFile);
    writetable(cmdDurationData, cmdDurationFile);
    writetable(positionData, positionFile);
    
    % Display progress
    fprintf('Processed folder: %s\n', folderName);
end

disp('All subfolders processed successfully.');