% Step 1: Generate the dataset
% Assuming all the necessary variables are defined before this step
%trajectory_dataset_name=['./',trajectory_dataset_name];
trajectory_dataset_name =["cellArray1_400_full.mat"];
% Parameters
struc=load(trajectory_dataset_name);
%cArray=struc.cellArray;
cArray=struc.cellArray;
% cArray=struc;
sizearray = size(cArray);
numSeq = sizearray(1); % Number of sequences
disp(numSeq)
%length of the testingdata
test_len=5;
% Function to apply to each matrix in modified_cArray
apply_function = @(matrix) subtract_lines_and_plot(matrix, 2); % Add the number of figures as an input

% Apply the function to each matrix in modified_cArray
modified_cArray = cellfun(apply_function, cArray, 'UniformOutput', false);

% Size treatment
numberofcells = numel(modified_cArray);
maxsize = size(modified_cArray{1}, 2);
multfactor = maxsize / test_len;

% Create a new cell array to store modified data
modifiedCellArray = modified_cArray;

% Process each cell in the original array
if multfactor ~= 1
    modifiedCellArray = cell(1, multfactor * maxsize);
    for k = 1:numberofcells
        % Get the data from the original cell
        originalCell = modified_cArray{k};
        for i = 1:multfactor
            acell = originalCell(:, (i - 1) * test_len + 1 : i * test_len);
           
            % Assign the cells to the modified cell array
            modifiedCellArray{(k - 1) * multfactor + i} = acell;
        end
    end
end

% Split the data into training and validation sets
totalCells = numel(modifiedCellArray);
index = round(0.8 * totalCells);
XTrain = modifiedCellArray(1:index);
XVal = modifiedCellArray(index+1:totalCells);

% %{for i = 1:numSeq
%     seqLength = 1000;  Random sequence length between 5 and maxSeqLength
%    XTrain{i} = randn(numFeatures, seqLength); % Random feature values
%     Ytr
%     end
%     %}

% Generate the pattern
pattern = mod(0:numSeq-1, numClasses);
% Create the categorical sequence
categoricalSequence = categorical(pattern, 0:numClasses-1);
% Repeat each category in categoricalSequence by multfactor times
repeatedSequence = repelem(categoricalSequence, multfactor);

% Split the labels into training and validation sets
totalElements = numel(repeatedSequence);
indexToKeep = round(0.8 * totalElements);
YTrain = repeatedSequence(1:indexToKeep);
YVal = repeatedSequence(indexToKeep+1:totalElements);

miniBatchSize = 64;
% Step 2: Define the neural network

inputSize = 6;
numHiddenUnits = 150;



layers = [
    sequenceInputLayer(inputSize, 'Name', 'inputFEN')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    bilstmLayer(numHiddenUnits, 'OutputMode', 'sequence')
    convolution1dLayer(2,5,'Stride',2,'Padding',1)
    maxPooling1dLayer(2,'Stride',3,'Padding',1)
    convolution1dLayer(5, 32, 'Padding', 'same', 'Stride', 2)
    globalAveragePooling1dLayer('Name', 'GlobalAveragePoolingfcn')
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
];

options = trainingOptions("adam", ...
    ExecutionEnvironment="gpu", ...
    GradientThreshold=1, ...
    MaxEpochs=100, ...
    MiniBatchSize=miniBatchSize, ...
    ValidationData={XVal,YVal}, ... %new
    ValidationFrequency=20, ...     %new
    SequenceLength="longest", ...
    L2Regularization = 0.0001, ...  %new
    Shuffle="once", ...
    Verbose=0, ...
    Plots="training-progress");

% options = trainingOptions("sgdm", ...
%     ExecutionEnvironment="cpu", ...  % Specify CPU execution
%     LearnRateSchedule="piecewise", ...
%     LearnRateDropFactor=0.2, ...
%     LearnRateDropPeriod=5, ...
%     MaxEpochs=200, ...
%     MiniBatchSize=128, ...
%     ValidationData={XVal,YVal}, ...
%     ValidationFrequency=20, ...
%     SequenceLength="longest", ...
%     L2Regularization = 0.0001, ...
%     Shuffle="once", ...
%     Verbose=0, ...
%     Plots="training-progress");


net = trainNetwork(XTrain,YTrain,layers,options);



save("2_class_differential",'net')
% Make predictions on the validation set
YPred = predict(net, XVal);

% Find the column index of the maximum probability for each row
[~, predictedClass] = max(YPred, [], 2);

% Create a categorical array from the predicted class indices
categoryNames = cellstr(num2str((0:max(predictedClass))'));  % Assuming classes are 0-based
categoricalPred = categorical(predictedClass - 1, 0:max(predictedClass), categoryNames);

% Compute confusion matrix
C = confusionmat(YVal, categoricalPred)

% Display confusion chart
figure
confusionchart(YVal, categoricalPred,'RowSummary','row-normalized');
title('Confusion Matrix');


precision = diag(C) ./ sum(C, 1)';
recall = diag(C) ./ sum(C, 2);
f1Score = 2 * (precision .* recall) ./ (precision + recall);


% Display the results
disp('Class   Precision   Recall   F1 Score');
disp([transpose(1:size(C, 1)), precision, recall, f1Score]);

% Create a new figure for ROC curves
figure

for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = ismember(YVal, num2str(i));
    
    % Extract predicted scores for the current class
    %YPredBinary =ismember(categoricalPred, num2str(i));
    YpredROC=YPred(:,i+1);
    
    % Compute ROC curve
    [X, Y, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1);
    
    % Plot ROC curve for the current class
    plot(X, Y, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    
    hold on;
end

% Add labels and legend
xlabel('False Positive Rate');
ylabel('True Positive Rate');
title('ROC Curves for Multi-Class Classification');
legend('show');

% Create a new figure for Precision-Recall curves
figure

for i = 0:numClasses-1
    % Convert true labels to binary
    YTruBinary = ismember(YVal, num2str(i));
    
    % Extract predicted scores for the current class
    YpredROC = YPred(:, i+1);
    
    % Compute Precision-Recall curve
    [precision, recall, ~, AUC] = perfcurve(YTruBinary, YpredROC, 1, 'xCrit', 'reca', 'yCrit', 'prec');
    
    % Plot Precision-Recall curve for the current class
    plot(recall, precision, 'DisplayName', ['Class ' num2str(i) ' (AUC = ' num2str(AUC) ')']);
    
    hold on;
end
% Add labels and legend
xlabel('Recall');
ylabel('Precision');
title('Precision-Recall Curves for Multi-Class Classification');
legend('Location', 'Best');
hold off; % Stop holding onto the current plot

function result_matrix = subtract_lines_and_plot(input_matrix, nb_figs)
    % Check if the input matrix has the correct dimensions
    if size(input_matrix, 1) ~= 6
        error('Input matrix must have 6 rows.');
    end
    
    % Resample the data: Replace every 1000 points with 100 points
    original_points = 1000;
    new_points = 5;
    resampled_matrix = zeros(size(input_matrix, 1), ceil(size(input_matrix, 2) * new_points / original_points));
    
    for i = 1:size(input_matrix, 1)
        % Use downsampling to replace every 1000 points with 100 points
        resampled_matrix(i, :) = downsample(input_matrix(i, :), original_points / new_points);
    end

    % Subtract the fourth, fifth, and sixth lines from the first, second, and third lines, respectively
    %result_matrix = resampled_matrix(4:6, :) - resampled_matrix(1:3, :);
    result_matrix = resampled_matrix;
    disp(size(result_matrix))
    % Plot the original lines
    a = rand;
    if a > 0.999
        figure;
        subplot(2, 1, 1);
        plot(1:size(resampled_matrix, 2), resampled_matrix(1, :));
        hold on;
        plot(1:size(resampled_matrix, 2), resampled_matrix(2, :));
        plot(1:size(resampled_matrix, 2), resampled_matrix(3, :));
        plot(1:size(resampled_matrix, 2), resampled_matrix(4, :));
        plot(1:size(resampled_matrix, 2), resampled_matrix(5, :));
        plot(1:size(resampled_matrix, 2), resampled_matrix(6, :));
        title('Original Lines');
        legend('Line 1', 'Line 2', 'Line 3', 'Line 4', 'Line 5', 'Line 6');
        hold off;

        % Plot the modified lines
        subplot(2, 1, 2);
        plot(1:size(result_matrix, 2), result_matrix(1, :), 'r');
        hold on;
        plot(1:size(result_matrix, 2), result_matrix(2, :), 'g');
        plot(1:size(result_matrix, 2), result_matrix(3, :), 'b');
        title('Modified Lines');
        legend('Line 1', 'Line 2', 'Line 3');
        hold off;
    end
end
