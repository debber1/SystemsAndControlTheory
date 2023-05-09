%% Clearing everything before the script runs
clear all;clc;close all;

%% Loading everything

% Setting time information
Tsample=1000;% Samples per second
Tend=0.2;
t=[0.001:1/Tsample:Tend]'; % Making time array

% Setting standard input array
u = ones(size(t,1),1); % Input array for system A
x = ones(size(t,1),1); % Input array for system B

% Loading the system
systemA = sim("transferfunction12A_harness.slx");
systemB = sim("transferfunction12B");

%% Question 1: Initial analysis of system A

% Plotting the stip response of system A
plot(systemA.tout, systemA.y, Color=[1, 0, 0]) % This plots the output of the system when we apply a step
title('Step response')
xlabel('Time')
ylabel('Amplitude')
% Making the bode plot of system A
[num, den]= linmod("transferfunction12A_harness"); % We make a linear model of the obfuscated model
%SysA = tf(num, den); % Make a transferfunction of the linmod results
step(SysA) % Plot the stepresponse (and compare to the step above as a sanity check)
%bode(SysA); % Plot the bode plot of the model

% Analysis of the system stability

%% Question 2: Reverse engineering of system A



%% Question 3: Analysis of system B



%% Question 4: Controll loop for system A



%% Question 5: Controll loop with delay for system A