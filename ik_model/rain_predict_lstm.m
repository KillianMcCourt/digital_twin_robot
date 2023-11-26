% Step 1: Generate the dataset
num_time_series = 6;
time_series_length = 6000;
num_classes = 7;

% Parameters
numSeq = 2800;      % Number of sequences
numFeatures = 7;   % Number of features
maxSeqLength = 1000; % Maximum sequence length

% Generate random sequences
%XTrain = cell(numSeq, 1);
YTrain = categorical(randi(6, numSeq, 1)); % Random labels from 1 to 6

% %{for i = 1:numSeq
%     seqLength = 1000;  Random sequence length between 5 and maxSeqLength
%    XTrain{i} = randn(numFeatures, seqLength); % Random feature values
%     Ytr
%     end
%     %}

numClasses = 7;  % Number of classes

% Generate categorical sequence
pattern = mod(0:numSeq-1, numClasses);
categoricalSequence = categorical(pattern, 0:numClasses-1);
disp(categoricalSequence);

XTrain = cellArray
YTrain = categoricalSequence
figure
plot(XTrain{1}')
xlabel("Time Step")
title("Training Observation 1")
%numFeatures = size(XTrain{1},1);
%legend("Feature " + string(1:numFeatures),Location="northeastoutside")
% Display the generated data

miniBatchSize = 32;
% Step 2: Define the neural network

inputSize = 6;
numHiddenUnits = 100;
numClasses = 7;

layers = [
    sequenceInputLayer(inputSize)
    bilstmLayer(numHiddenUnits, 'OutputMode', 'last')
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
];

options = trainingOptions("adam", ...
    ExecutionEnvironment="cpu", ...
    GradientThreshold=1, ...
    MaxEpochs=50, ...
    MiniBatchSize=miniBatchSize, ...
    SequenceLength="longest", ...
    Shuffle="never", ...
    Verbose=0, ...
    Plots="training-progress");


net = trainNetwork(XTrain,YTrain,layers,options);

