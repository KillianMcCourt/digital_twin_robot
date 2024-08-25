% Iterate over each element in the cell array
%%
for i = 1:10 %length(cell_array)
    data = simulated_cell_dataset{i}; % Extract data for the current cell
    disp(size(data))
    data = data(:,200:end);
    %pre_downsample = classify(net, data)
    %data = data(:, 1:10:end);
    %post_downsample = classify(net, data)
    % Create a new figure for each iteration
    figure;

    % Plot the first and fourth lines
    subplot(3, 1, 1); % Create the first subplot
    plot(data(1, :), 'r', 'DisplayName', 'Command'); % Plot the first line in red
    hold on; % Hold the plot for adding more data
    plot(data(4, :), 'b', 'DisplayName', 'Response'); % Plot the fourth line in blue
    hold off;
    legend; % Show the legend
    title(sprintf('Iteration %d: Lines 1 and 4', i)); % Title for the subplot
    xlabel('Sample'); % Label for the x-axis
    ylabel('Value'); % Label for the y-axis

    % Plot the second and fifth lines
    subplot(3, 1, 2); % Create the second subplot
    plot(data(2, :), 'r', 'DisplayName', 'Command'); % Plot the second line in red
    hold on;
    plot(data(5, :), 'b', 'DisplayName', 'Response'); % Plot the fifth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 5', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot the third and sixth lines
    subplot(3, 1, 3); % Create the third subplot
    plot(data(3, :), 'r', 'DisplayName', 'Command'); % Plot the third line in red
    hold on;
    plot(data(6, :), 'b', 'DisplayName', 'Response'); % Plot the sixth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 6', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Adjust subplot spacing
    sgtitle(sprintf('Iteration %d, real class %d, ai prediction %d', i, mod(i,9), classify(net, data) )); % Add a super-title for the whole figure
end

%%
for i = 1:9 %length(cell_array)
    data = new_real_data{1, i}; % Extract data for the current cell

    data = data(:,200:end)
    %pre_downsample = classify(net, data)
    %data = data(:, 1:10:end);
    %post_downsample = classify(net, data)
    % Create a new figure for each iteration
    figure;

    % Plot the first and fourth lines
    subplot(3, 1, 1); % Create the first subplot
    plot(data(1, :), 'r', 'DisplayName', 'Command'); % Plot the first line in red
    hold on; % Hold the plot for adding more data
    plot(data(4, :), 'b', 'DisplayName', 'Response'); % Plot the fourth line in blue
    hold off;
    legend; % Show the legend
    title(sprintf('Iteration %d: Lines 1 and 4', i)); % Title for the subplot
    xlabel('Sample'); % Label for the x-axis
    ylabel('Value'); % Label for the y-axis

    % Plot the second and fifth lines
    subplot(3, 1, 2); % Create the second subplot
    plot(data(2, :), 'r', 'DisplayName', 'Command'); % Plot the second line in red
    hold on;
    plot(data(5, :), 'b', 'DisplayName', 'Response'); % Plot the fifth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 5', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot the third and sixth lines
    subplot(3, 1, 3); % Create the third subplot
    plot(data(3, :), 'r', 'DisplayName', 'Command'); % Plot the third line in red
    hold on;
    plot(data(6, :), 'b', 'DisplayName', 'Response'); % Plot the sixth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 6', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Adjust subplot spacing
    sgtitle(sprintf('Iteration %d, real class %d, ai prediction %d', i, mod(i,9), classify(net, data) )); % Add a super-title for the whole figure
end

%% for 8d data

% Itera1e over the length of the cell array, adjust the range as necessary
for i = 1:10 % Update this as per your dataset's length

    % Extract data for the current cell, transpose if necessary
    data = post_ik_data_real_cell{i, 1};

    % Display the size of the data for debugging purposes
    disp(size(data))

    % Extract relevant data starting from the 200th column
    data = data(:, 200:end);

    % Create a new figure for each iteration
    figure;

    % Plot lines 1 and 5
    subplot(4, 1, 1); % Create the first subplot
    plot(data(1, :), 'r', 'DisplayName', 'Command'); % Plot the first line in red
    hold on; % Hold the plot for adding more data
    plot(data(5, :), 'b', 'DisplayName', 'Response'); % Plot the fifth line in blue
    hold off;
    legend; % Show the legend
    title(sprintf('Iteration %d: Lines 1 and 5', i)); % Title for the subplot
    xlabel('Sample'); % Label for the x-axis
    ylabel('Value'); % Label for the y-axis

    % Plot lines 2 and 6
    subplot(4, 1, 2); % Create the second subplot
    plot(data(2, :), 'r', 'DisplayName', 'Command'); % Plot the second line in red
    hold on;
    plot(data(6, :), 'b', 'DisplayName', 'Response'); % Plot the sixth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 2 and 6', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot lines 3 and 7
    subplot(4, 1, 3); % Create the third subplot
    plot(data(3, :), 'r', 'DisplayName', 'Command'); % Plot the third line in red
    hold on;
    plot(data(7, :), 'b', 'DisplayName', 'Response'); % Plot the seventh line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 3 and 7', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

    % Plot lines 4 and 8
    subplot(4, 1, 4); % Create the fourth subplot
    plot(data(4, :), 'r', 'DisplayName', 'Command'); % Plot the fourth line in red
    hold on;
    plot(data(8, :), 'b', 'DisplayName', 'Response'); % Plot the eighth line in blue
    hold off;
    legend;
    title(sprintf('Iteration %d: Lines 4 and 8', i)); % Title for the subplot
    xlabel('Sample');
    ylabel('Value');

 

end