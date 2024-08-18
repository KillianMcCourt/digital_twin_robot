%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data

samplePeriod = 1/100;
data = readMatData('2024-05-24\09-52-40-302\Matlab\data.mat');
same_size = size(data,1);
acc = data(1:same_size,3:5);
acc(:,3) = -acc(:,3);
acc(:,2)=-acc(:,2);
gyr = data(1:same_size,6:8);
quatern_imu = data(1:same_size,27:30);  

%% Process data through AHRS algorithm (calcualte orientation)

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
ahrs.Quaternion = quatern_imu(1,:);
for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), -acc(i,:));	% gyroscope units must be radians
    R(:,:,i) = -quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer
% accelerometer in Earth frame
tcAcc = zeros(size(acc));  
for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

% Calculate linear acceleration in Earth frame (subtracting gravity)
linAcc = tcAcc -[zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s
%% low-pass filter and smoothing
cutoff_frequency = 20; % Hz
[b_low, a_low] = butter(4, cutoff_frequency/(1/samplePeriod/2), 'low');
order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linAcc = filtfilt(b, a, linAcc);

%% FK data and command

df = readmatrix("comparison_matrix.csv");
FK_freq = 10;
FK_Pos = zeros(size(df,1),3);
FK_Pos(:,1) = df(:,4);
FK_Pos(:,2) = df(:,5);
FK_Pos(:,3) = df(:,6);
command = df(:,1:3);
FK_v = diff(FK_Pos)*100;

%% align start time of sensor and KF result
window_size = 50; 
energy = movmean(sum(gyr,2)'.^2, window_size);

threshold = 1.5 * mean(energy); % set energy threshold 
start_idx_sensor = find(energy > threshold, 1, 'first');


diff_signal = diff(command);
% set up threshold to detect the variation of the signal
threshold = 1e-4; % threshold
start_idx_command = find(abs(diff_signal) > threshold, 1, 'first') + 1; % find the index of variation

linAcc = linAcc(start_idx_sensor - start_idx_command:end,:);


%% ESKF
%initial kalman filter

linVel = zeros(size(linAcc));
linPos = zeros(size(linVel));
linPos(1,:) = FK_Pos(1,:);
dx = zeros(9,1);
Pk = 100*diag([1*ones(1,3),1*ones(1,3),1*ones(1,3)]);  %variance of error state
Gk = [eye(6) zeros(6,3)]; % equation of observation
Ck = eye(6);
Q = diag([1*ones(1,3),1*ones(1,3)]);%variance of IMU noise of acceleration and acc_bias
Rk = (0.01*FK_freq/samplePeriod)^2*eye(6);%FK noise

Fk = eye(9) + samplePeriod*[zeros(3,3) eye(3) zeros(3,3);zeros(3,6) -1*eye(3);zeros(3,9)];  %equation of state
Bk = [zeros(3,6);eye(3)*1/samplePeriod zeros(3,3);zeros(3,3) eye(3)*1/samplePeriod^0.5];

bias_acc = zeros(size(linAcc)); 

FK_idx = 1;
for i = 2:length(linAcc)
    % Position predict by IMU
    bias_acc(i,:) = bias_acc(i-1,:);
    linVel(i,:) = linVel(i-1,:) + (linAcc(i,:)-bias_acc(i,:)+linAcc(i-1,:)-bias_acc(i-1,:))/2 * samplePeriod;
    linPos(i,:) = linPos(i-1,:) + 0.5*(linVel(i,:)+linVel(i-1,:)) * samplePeriod;
    %Update Kalman prediction
    wk = [0*randn(3,1);0*randn(3,1)];
    dx = Fk *dx +Bk*wk; 
    Pk = Fk*Pk*Fk'+Bk*Q*Bk';
    if mod(i, 1/samplePeriod / FK_freq) == 0 && FK_idx*10 < size(FK_Pos,1)  
        Kk = Pk*Gk'*pinv(Gk*Pk*Gk'+Ck*Rk*Ck'); 
        Pk = (eye(9)-Kk*Gk)*Pk;
        dx = dx + Kk*([linPos(i,:) linVel(i,:)]'-[FK_Pos(FK_idx*10,:) FK_v(FK_idx*10,:)]'-Gk*dx);
        FK_idx = FK_idx + 1;
        linPos(i,:) = linPos(i,:)- dx(1:3)';
        linVel(i,:) = linVel(i,:) - dx(4:6)';
        bias_acc(i,:) = bias_acc(i,:) - dx(7:9)';
        dx=zeros(9,1);
    end
end
linPos_akl = zeros(size(FK_Pos));
linPos_akl(1,:) = FK_Pos(1,:);
for i = 2:length(linPos_akl)    
    linPos_akl(i,:) = linPos_akl(i-1,:) + 0.5*(linVel(i,:)+linVel(i-1,:)) * samplePeriod;
end


%% Plot
figure
subplot(1,2,1)

hold on;
plot(command(:,1),'r');
plot(command(:,2),'g');
plot(command(:,3),'b');
title('command');


subplot(1,2,2)
hold on;
plot(linPos_akl(:,1), 'r');
plot(linPos_akl(:,2), 'g');
plot(linPos_akl(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position');
legend('X', 'Y', 'Z');
%%
% Plot
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
subplot(3,3,7)
hold on;
plot(gyr(:,1), 'r');
plot(gyr(:,2), 'g');
plot(gyr(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

subplot(3,3,4)
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');


subplot(3,3,1)
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');
subplot(3,3,8)
hold on;
plot(FK_Pos(:,1),'r');
plot(FK_Pos(:,2),'g');
plot(FK_Pos(:,3),'b');
% Plot
subplot(3,3,2)
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');
% Plot
subplot(3,3,3)
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position');
legend('X', 'Y', 'Z');
%Plot velocity without kalman filter
linVel_bkf = zeros(size(linAcc));
linPos_bkf = zeros(size(linVel));
linPos_bkf(1,:)=FK_Pos(1,:);
for i = 2:length(linAcc)
    linVel_bkf(i,:) = linVel_bkf(i-1,:) + (linAcc(i,:)+linAcc(i-1,:))/2 * samplePeriod;
    linPos_bkf(i,:) = linPos_bkf(i-1,:) + 0.5*(linVel_bkf(i,:)+linVel_bkf(i-1,:)) * samplePeriod;
end
subplot(3,3,5)
hold on;
plot(linVel_bkf(:,1), 'r');
plot(linVel_bkf(:,2), 'g');
plot(linVel_bkf(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear velocity without kalman filter');
legend('X', 'Y', 'Z');
subplot(3,3,6)
hold on;
plot(linPos_bkf(:,1), 'r');
plot(linPos_bkf(:,2), 'g');
plot(linPos_bkf(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position without kalman filter');
legend('X', 'Y', 'Z');
%Plot
linPos_akl = zeros(size(FK_Pos));
linPos_akl(1,:) = FK_Pos(1,:);
for i = 2:length(linPos_akl)    
    linPos_akl(i,:) = linPos_akl(i-1,:) + 0.5*(linVel(i,:)+linVel(i-1,:)) * samplePeriod;
end
subplot(3,3,9)
hold on;
plot(linPos_akl(:,1), 'r');
plot(linPos_akl(:,2), 'g');
plot(linPos_akl(:,3), 'b');
xlabel('sample');
ylabel('m');
title('Linear position');
legend('X', 'Y', 'Z');