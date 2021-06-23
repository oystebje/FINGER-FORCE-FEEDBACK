%% Playing with strategies for simulating cutting of meniscus
%(and some analysis of cut data)

% By Øystein Bjelland, CPS Lab, IIR, NTNU

clear;
clc;
close all;

%% Simulation

% Declaring variables and constants
dtheta = 0.01;
theta_o = 0:dtheta:12; %Define theta as a list from 0 to 12 degrees.
theta_o = [theta_o theta_o];
theta = theta_o;
L = length(theta_o);

for i = 1:(L/2-1)
       theta((L/2)+i) = theta_o((L/2)-i);
end
theta(L) = 0;

len = length(theta);
force = zeros(1, len);

interceptionPoint = 4.8; %[deg]
yieldPoint = 591; % [force]
yieldPoint_d = 6; % [deg]

bottomOutPoint = 516; % [force]
bottomOutPoint_d = 12; %[deg]

openingPoint = 789; % [force]
openingPoint_d = 11; %[deg]


%dF = 0;

% Elastic regime
K1 = 450; % Stiffness in [no value/deg]
% Plastic regime
K2 = 20;
% Bottom-out regime
K3 = 300;
% Opening regime
K4 = 1600;

%% Simulation


for i = 2:len
    
    if ((theta(i) > interceptionPoint) && (theta(i) < yieldPoint_d) && (dtheta > 0))
        force(i) = force(i-1) + dtheta*K1;
        %fprintf('reached interception point. \n');
        
    elseif ((theta(i) >= yieldPoint_d) && (theta(i) <= bottomOutPoint_d) && (dtheta > 0)) 
        force(i) = force(i-1) - dtheta*K2;
        %fprintf('reached yield point. \n');
        
    elseif ((theta(i) <= bottomOutPoint_d) && (theta(i) >= openingPoint_d) && (dtheta < 0))
        force(i) = force(i-1) - dtheta*K3;     
        %fprintf('reached bottomout point. \n');
        
    elseif ((theta(i) < openingPoint_d) && (dtheta < 0))
        force(i) = force(i-1) + dtheta*K4; 
        %fprintf('reached opening point. \n');
        
        if force(i) < 0
            force(i) = 0;
        end
        
%     elseif ((theta(i) < openingPoint_d) && (dtheta < 0) && (theta(i) > 8.3))
%         force(i) = force(i-1) + dtheta*K4; 
%         fprintf('reached opening point. \n');

    else
        force(i) = 0;
        fprintf('no force. \n');
    end
    
    dtheta = theta(i) - theta(i-1);
    
end
   
   
%% Importing cutting data for comparison

filename1 = '11_meniscus_fast_1826 - Copy.txt';
%filename2 = '2_blank_fast_1759.txt';
filename2 = '2_blank_fast_1759_copy_.txt';

A = importdata(filename1);
B = importdata(filename2);

inputAngle_raw = A(:,4);  %Input angle [deg]
outputFingerForce_raw = A(:,1);   %Output finger force [raw fsr reading]. Set to ":2" for [g]

inputAngle_raw_blank = B(:,4);  %Input angle [deg]
outputFingerForce_raw_blank = B(:,1);   %Output finger force [raw fsr reading]. Set to ":2" for [g]

time_raw = A(:,5);  %Raw time from millis() in Arduino [milliseconds]
time = zeros(length(time_raw),1);
Ts_vect = [];

Ts_ref = 5*(time_raw(3) - time_raw(2));

for i = 2:length(time_raw)
  
   Ts = time_raw(i) - time_raw(i-1);
   
   % Preventing the time gap between the sampling series to artificially increase sampling time.
   if Ts <  Ts_ref
        Ts_vect = [Ts_vect, Ts];
   end
   
   time(i) = (time_raw(i) - time_raw(1))/1000; %Starting the time vector from zero and converting from ms to s.
   
end

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

disp('Our sampling frequency is, Fs [Hz]')
Fs = 1/Ts;
disp(Fs)

inputAngle_filtered = lowpass(inputAngle_raw, 5, Fs);
inputAngle_filtered_blank = lowpass(inputAngle_raw_blank, 5, Fs);

%% For plotting selected cut

theta_first_cut = inputAngle_filtered(1:250);
Force_first_cut = outputFingerForce_raw(1:250);

%theta_random_cut = inputAngle_filtered(1:250);
%Force_random_cut = outputFingerForce_raw(1:250);

%theta_random_cut = inputAngle_filtered(870:1000);
%Force_random_cut = outputFingerForce_raw(870:1000);

%theta_random_cut = inputAngle_filtered(1400:1600);
%Force_random_cut = outputFingerForce_raw(1400:1600);

%theta_random_cut = inputAngle_filtered(2300:2500);
%Force_random_cut = outputFingerForce_raw(2300:2500);

%theta_random_cut = inputAngle_filtered(3000:3200);
%Force_random_cut = outputFingerForce_raw(3000:3200);

%theta_random_cut = inputAngle_filtered(3700:3900);
%Force_random_cut = outputFingerForce_raw(3700:3900);

%theta_random_cut = inputAngle_filtered(4300:4500);
%Force_random_cut = outputFingerForce_raw(4300:4500);

%theta_random_cut = inputAngle_filtered(5100:5300);
%Force_random_cut = outputFingerForce_raw(5100:5300);

theta_random_cut = inputAngle_filtered(5900:6100);
Force_random_cut = outputFingerForce_raw(5900:6100);

%theta_random_cut = inputAngle_filtered(6800:6950);
%Force_random_cut = outputFingerForce_raw(6800:6950);

%% Select a blank cut for comparison
theta_blank_random_cut = inputAngle_filtered_blank(2600:3000);
Force_blank_random_cut = outputFingerForce_raw_blank(2600:3000);

%% Converting raw force data to newtons

outputFingerForce_newtons = zeros(1, length(outputFingerForce_raw));
blank_newtons = zeros(1, length(outputFingerForce_raw_blank));

outputFingerForce_blank_cal = outputFingerForce_raw_blank - 137; %Correcting for offset

for i = 1:length(outputFingerForce_raw)    
    % Conversion from calibration data in excel sheet, plus converting from
    % gram to newton
    outputFingerForce_newtons(i) = (3*10^(-7)*(outputFingerForce_raw(i))^3 - 0.0002*(outputFingerForce_raw(i))^2 + 0.7815*outputFingerForce_raw(i))*(10^(-3)*9.81);
    
end

for i = 1:length(outputFingerForce_raw_blank)
    blank_newtons(i) = (3*10^(-7)*(outputFingerForce_blank_cal(i))^3 - 0.0002*(outputFingerForce_blank_cal(i))^2 + 0.7815*outputFingerForce_blank_cal(i))*(10^(-3)*9.81);
end

Force_random_cut_newtons = outputFingerForce_newtons(5900:6100);

%% Importing render for comparison

filename3 = 'haptic_render_25052021.txt';
C = importdata(filename3);

theta_haptic = C(:,1);
theta_voltage = C(:,2);
voltage_haptic = theta_voltage*(-1);

for i = 1:length(theta_haptic)
    if (theta_haptic(i) < 0)
        theta_haptic(i) = 0;
    elseif (theta_haptic(i) > 12)
        theta_haptic(i) = 12;
    end
end
%%

figure(1)
%subplot(3,1,1)
%plot(theta,force, 'b');
%title('Simulated force curve');

subplot(2,1,1)
plot(inputAngle_filtered, outputFingerForce_newtons,'k'); 
title('Partial Meniscectomy Punch Force Curve');
xlabel('Puncher Angle [degrees]');
ylabel('F_{FSR} [N]');

subplot(2,1,2)
plot(inputAngle_filtered_blank, blank_newtons, 'k');
title('Blank Cut Force Curve');
xlabel('Puncher Angle [degrees]');
ylabel('F_{FSR} [N]');




% figure(2)
% subplot(3,1,1)
% plot(inputAngle_filtered, outputFingerForce_raw,'r');
% title('Measured data')
% 
% subplot(3,1,2)
% plot(theta_blank_random_cut, Force_blank_random_cut, 'g');
% title('Blank cut');
% 
% subplot(3,1,3)
% plot(theta_random_cut, Force_random_cut);
% title('Plot of selected cut');
%
% figure(3)
% plot(theta_random_cut, Force_random_cut);
% %title('Plot of meniscus cut #6, in test #11');
% title('Meniscectomy Punch Force Curve');
% xlabel('Puncher Angle [degrees]');
% ylabel('Finger Force [no unit]');
% 
% figure(4)
% plot(theta, force)
% 
% 
% figure(5)
% 
% subplot(3,1,1)
% plot(theta_random_cut, Force_random_cut);
% title('Plot of selected cut');
% 
% subplot(3,1,2)
% plot(theta, force)
% title('Simulated force curve');
% 
% subplot(3,1,3)
% plot(theta_haptic, voltage_haptic)
% title('Rendered force curve');
% 
% 
figure(6)
plot(theta_random_cut, Force_random_cut_newtons);
title('Meniscectomy Punch Force Curve');
xlabel('Puncher Angle [deg]');
ylabel('Finger Force [N]');