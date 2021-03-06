% Kalibrering av FSR

clear;
clc;
close all;

%10kOhm resistor

%% Bare for sjekk mot adafruit eksempel!

%Vekt_ref = (1000/9.81)*[0.2 1 10 100];
%FSR_ref = (1024/5) * [1.3 3.1 4.5 4.9];

%%

Vekt_ref = [0 100 200 300 400 500 600 1200];
FSR_ref = [1 900 940 950 970 970 978 984];

fsrVoltage = [];    % Millivolts
fsrResistance = []; % Ohm
fsrConductance = []; % Micromohs
fsrForce = []; %Newton
fsrGram = []; %gram
fsrError = [];  %gram

for i = 1:length(FSR_ref)
    fsrVoltage(i) = FSR_ref(i)*(5000/1024);
    fsrResistance(i) = ((5000 - fsrVoltage(i))*10000)/ fsrVoltage(i);
    fsrConductance(i) = 1000000 / fsrResistance(i);
    
    if fsrConductance(i) <= 1000 %eg 1000
        fsrForce(i) = fsrConductance(i)/280;
        fsrGram(i) = (fsrForce(i)/9.81)*1000;
    else
        fsrForce(i) = (fsrConductance(i) - 1000)/140;
        %fsrForce(i) = (fsrConductance(i) - 1000)/80;
        fsrGram(i) = (fsrForce(i)/9.81)*1000;
    end
    
end

fsrError = fsrGram - Vekt_ref;

%%
% Større resistor. Ikke relevant.

%Vekt_ny = [0 100 200 300 400 500 600 700 800 900 1000];
%FSR_ny = [0 240 320 430 400 440 480 530 550 630 640];

%%

figure(1)

subplot(2,1,1) 
plot(Vekt_ref,'r'); 
hold on
plot(FSR_ref,'b');
hold on
plot(fsrGram,'g');
hold on
plot(fsrConductance);

subplot(2,1,2)
plot(fsrError);
