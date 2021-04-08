

%% System identification of cutting in steak with arthroscopic grasper
% By Øystein Bjelland, IIR, NTNU

clear;
clc;
close all;

%% Import file into Matlab. File should be in txt-format.
filename1 = 'Pilot2_finger_26012021_Data_filtrert.txt'; %File must be located in same folder
A = importdata(filename1);

filename2 = 'Pilot2_finger_26012021_Validation_Data.txt';
B = importdata(filename2);

filename3 = 'Pilot2_finger_26012021_cutdata - Copy.txt';
C = importdata(filename3);


%% We need to rearrange the data on the form iddata(output,input,sampling time)

inputAngle_raw = A(:,2);  %Input angle [deg]
outputFingerForce_raw = A(:,1);   %Output finger force [N]
time_raw = A(:,3);  %Raw time from millis() in Arduino [milliseconds]

time = zeros(length(time_raw),1);
Ts_vect = [];

%% Same for validation and cut data

inputAngle_val = B(:,2);
outputFingerForce_val = B(:,1);

inputAngle_cut = C(:,2);
outputFingerForce_cut = C(:,1);


%% First, we check the sampling time

% Set up a reference sample time to make sure the order of magnitude of our
% sampling time is correct.
Ts_ref = 5*(time_raw(3) - time_raw(2));

for i = 2:length(time_raw)
  
   Ts = time_raw(i) - time_raw(i-1);
   
   % Preventing the time gap between the sampling series to artificially increase sampling time.
   if Ts <  Ts_ref
        Ts_vect = [Ts_vect, Ts];
   end
   
   time(i) = (time_raw(i) - time_raw(1))/1000; %Starting the time vector from zero and converting from ms to s.
   
end

maxTs = max(Ts_vect);
disp('Maximum sample time [ms]: ')
disp(maxTs)

minTs = min(Ts_vect);
disp('Minimum sample time [ms]: ') 
disp(minTs)

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

%% Now, lets structure our data as iddata

FINGER_FORCE_DATA_1 = iddata(outputFingerForce_raw, inputAngle_raw, Ts);
FINGER_FORCE_DATA_VALIDATION = iddata(outputFingerForce_val, inputAngle_val, Ts);
FINGER_FORCE_DATA_3 = iddata(outputFingerForce_cut, inputAngle_cut, Ts);

%% Visualize the data before starting system identification

figure(1)

subplot(2,1,1)
plot(time, inputAngle_raw)
grid on
xlabel('time [s]')
ylabel('Angle (deg)')

subplot(2,1,2)
plot(time, outputFingerForce_raw)
grid on
xlabel('time [s]')
ylabel('Finger Force [no unit]')

