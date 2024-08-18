% Assuming the pre-trained network is already loaded as 'net'



% Training options for transfer learning
optionsTransfer = trainingOptions("adam", ...
    ExecutionEnvironment="gpu", ...
    GradientThreshold=1, ...
    InitialLearnRate=0.0001, ...  % Lower learning rate for fine-tuning
    MaxEpochs=600, ...  % Fewer epochs for fine-tuning
    MiniBatchSize=miniBatchSize, ...
    ValidationData={XValReal, YValReal}, ...
    ValidationFrequency=20, ...
    SequenceLength="longest", ...
    L2Regularization=0.001, ...
    Shuffle="once", ...
    Verbose=0, ...
    Plots="training-progress");

% Train the network on the real data
netTransfer = trainNetwork(XTrainReal, YTrainReal, net.Layers, optionsTransfer);

% Evaluate the model on validation data
YPred = classify(netTransfer, XValReal);
accuracy = mean(YPred == YValReal);
disp("Validation Accuracy: " + accuracy*100 + "%");

YPred = classify(netTransfer, XValReal);

% Generate the confusion matrix
confusionMatrix = confusionchart(YValReal, YPred);

% Customize the confusion matrix
confusionMatrix.Title = 'Confusion Matrix for Validation Data';
confusionMatrix.RowSummary = 'row-normalized';
confusionMatrix.ColumnSummary = 'column-normalized';

% Display accuracy
accuracy = mean(YPred == YValReal);
disp("Validation Accuracy: " + accuracy*100 + "%");
