clear;
clc;
close all;

%%

filename1 = 'haptic_render_4_09062021.txt';
%filename1 = 'haptic_render_20062021.txt';
A = importdata(filename1);

%%

voltage = A(:,1); 
voltage = voltage*(-1);
pos = A(:,2);   
vel = A(:,3);  
time_raw = A(:, 4);

time = zeros(length(time_raw),1);
Ts_vect = [];

%%

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

disp('Our sampling frequency is, Fs [Hz]')
Fs = 1/Ts;
disp(Fs)


%%

i_start = 3700;
i_fin = 4350;

time_sel = time(i_start:i_fin);
voltage_sel = voltage(i_start:i_fin);
pos_sel = pos(i_start:i_fin);
vel_sel = vel(i_start:i_fin);


%%

figure(1)
subplot(2,1,1)
plot(time, pos, 'k')
title('Simulated Punch Force Curve')

xlabel('time [s]')
ylabel('Angle (deg)')

subplot(2,1,2)
plot(time, voltage, 'k')

xlabel('time [s]')
ylabel('Voltage [V]')


figure(2)
subplot(3,1,1)
title('selected cut in plastic region')
plot(time_sel, voltage_sel, 'k')
ylabel('Voltage [V]')

subplot(3,1,2)
plot(time_sel, pos_sel, 'k')
ylabel('pos [deeg]')

subplot(3,1,3)
plot(time_sel, vel_sel, 'k')
ylabel('velocity [deg/s]')
xlabel('time [ms]')

figure(3)
plot(pos, voltage, 'k')
title('Partial Meniscectomy Render Voltage Curve')
ylabel('Voltage [V]')
xlabel('Angle [deg]')
axis([0 13 0 13]) 