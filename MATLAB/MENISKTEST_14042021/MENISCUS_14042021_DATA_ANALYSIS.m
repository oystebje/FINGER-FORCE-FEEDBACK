%% Some data analysis from test 14042021

% By Øystein Bjelland, CPS Lab, IIR, NTNU

clear;
clc;
close all;

%% We are looking at test #11

interceptionPoint = [3.4 3.8 3.8 4.3 4.3 4.8 4.9 5.3 5.4 7.7];

yieldPoint = [526 668 548 666 549 626 705 614 531 480];

K = [574.49 345.61 358.17 479.80 584.22 339.12 324.99 502.75 297.43 534.33]; 


%% Plots

figure(1)

plot(interceptionPoint, yieldPoint, 'r');
hold on
plot(interceptionPoint, K, 'b');
legend('Yield Point [force]', 'Stiffness [force/degree]');
xlabel('Interception Point [degrees]');



