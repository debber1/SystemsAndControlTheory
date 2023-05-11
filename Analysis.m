%% Clearing everything before the script runs
clear all;clc;close all;

%% Loading everything

load('data.mat')

% Setting time information
Tsample=10000;% Samples per second
Tend=0.15;
t=[0.001:1/Tsample:Tend]'; % Making time array

% Setting standard input array
u = ones(size(t,1),1); % Input array for system A

% Loading the system
systemA = sim("transferfunction12A_harness.slx");
%% Question 1: Initial analysis of system A

% Plotting the step response of system A
% figure(1)
% plot(systemA.tout, systemA.y) % This plots the output of the system when we apply a step
% title('Step response')
% xlabel('Time')
% ylabel('Amplitude')

% Making the bode plot of system A
load = true;
if load == false % We add the option to load the data from memory, because this analysis takes a while. 
    samples = 500;
    step = 2;
    [phaseShifts, amplitudeRatios, frequency] = generateBode(samples, step, Tsample, Tend, t);
end
% figure(2)
% subplot(2,1,1)
% semilogx(frequency, mag2db(amplitudeRatios))
% yline(0)
% xline(216.7)
% title('Magnitude response')
% xlabel('frequency (RAD/s)')
% ylabel('Amplitude (dB)')
% subplot(2,1,2)
% semilogx(frequency, phaseShifts)
% xline(216.7)
% yline(-93.12)
% title('Phase response')
% xlabel('frequency (RAD/s)')
% ylabel('Phase (Degrees)')

% Analysis of the system stability

%% Question 2: Reverse engineering of system A

% Get more info about the step response
systemA = sim("transferfunction12A_harness.slx");
%stepinfo(systemA.y, systemA.tout)

% Finding poles through the resonance
[peakAmplitudeRatio, peakAmplitudeRatioIndex] = max(amplitudeRatios);
DCamplitudeRatio = amplitudeRatios(1);
Q = peakAmplitudeRatio/DCamplitudeRatio;
zeta = 1/(2*Q);

peakFrequency = frequency(peakAmplitudeRatioIndex);
naturalFrequency = peakFrequency/(sqrt(1-2*zeta^2));

zeroGuess = 540;  % here is a guess of what the value is for the zero in the system
tau_z = 1/zeroGuess;

numerator = (DCamplitudeRatio * naturalFrequency^2).*[tau_z, 1];
denominator = [1, 2*zeta*naturalFrequency, naturalFrequency^2];
REsystemA = tf(numerator, denominator)

%Compare the reverse engineer to the original
figure(3) % Compare bode plots (original left, RE right)
subplot(2,2,1)
semilogx(frequency, mag2db(amplitudeRatios))
title('Magnitude response (system)')
xlabel('frequency (RAD/s)')
ylabel('Amplitude (dB)')
subplot(2,2,3)
semilogx(frequency, phaseShifts)
title('Phase response (system)')
xlabel('frequency (RAD/s)')
ylabel('Phase (Degrees)')
subplot(2,2,2)
bodemag(REsystemA)
subplot(2,2,4)
h = bodeplot(REsystemA);
setoptions(h,'MagVisible','off');

figure(4)
step(REsystemA);
hold on
plot(systemA.tout, systemA.y, Color = [1,0,0]) % This plots the output of the system when we apply a step
title('Step response')
xlabel('Time')
ylabel('Amplitude')
legend('Reverse engineered step', 'Simulated step')

figure(8)
margin(REsystemA)

%% Question 3: Analysis of system B
% Defining system B
numeratorSystemB = [183, 7247];
denominatorSystemB = [1, -119, 10816];
TFsystemB = tf(numeratorSystemB, denominatorSystemB);

% Plots needed for stability analysis
% figure(5)
% step(TFsystemB)
% figure(6)
% bode(TFsystemB)
% figure(7)
% rlocus(TFsystemB)
% stepinfo(TFsystemB)
% figure(8)
% pzmap(TFsystemB)
% figure(9)
% margin(TFsystemB)

% Adding a P controller to stabilize the system
% Defining sim variables
K = 0.6502732; % I found this by guessing 
EX3_SENS_NUM = [1];
EX3_SENS_DEN = [1];
% Setting time information
Tsample=10000;% Samples per second
Tend=0.151;
%Tend= 20;
t=[0.001:1/Tsample:Tend]'; % Making time array
x = ones(size(t,1),1); % Input array for system B

PcontrolSystemB = sim("transferfunction12B_Pcontrol.slx", t);

% figure(10)
% plot(PcontrolSystemB.tout, PcontrolSystemB.y);
% ylabel("Amplitude")
% xlabel("Time (s)")
% title("Step response controlled system B")
% Tuning the P controller to be critically damped
% For critical damping we want zeta = 1
K = db2mag(-3.74); %We get this from the gain margin (this is marginal stability)
%K = 2.51; %rlocus does not work because of a zero
[omegan, zeta] = damp(TFsystemB)

K = 4*omegan(1)^2; % This is supposed to make zeta = 
K = 50000;
PcontrolSystemB = sim("transferfunction12B_Pcontrol.slx", t);

figure(11)
plot(PcontrolSystemB.tout, PcontrolSystemB.y);
stepinfo(PcontrolSystemB.y, PcontrolSystemB.tout)

%% Question 4: Control loop for system A

% Setting time information
Tsample=10000;% Samples per second
Tend=0.1;
t=[0.001:1/Tsample:Tend]'; % Making time array
u = ones(size(t,1),1); % Input array for system A
T_i = 0.02;
T_d = 0;
K_P = 1;
KC = 70;
T_DELAY = 0;

EX4_SENS_NUM = [1];
EX4_SENS_DEN = [1];

ControlSystemA = sim("transferfunction12A_Control.slx");

figure(12)
plot(ControlSystemA.tout, ControlSystemA.y);
title("Step response control loop system A")
ylabel("Amplitude")
xlabel("time (s)")
stepinfo(ControlSystemA.y, ControlSystemA.tout)

%% Question 5: Control loop with delay for system A


% Setting time information
Tsample=10000;% Samples per second
Tend=0.5;
t=[0.001:1/Tsample:Tend]'; % Making time array
u = ones(size(t,1),1); % Input array for system A
T_i = 0.001;
T_d = 30;
K_P = 1;
KC = 0.05;
T_DELAY = 0.006996183416; % This gets calculated with gain margin data

EX4_SENS_NUM = [1];
EX4_SENS_DEN = [1];

ControlSystemA = sim("transferfunction12A_Control.slx");

figure(12)
plot(ControlSystemA.tout, ControlSystemA.y);
title("Step response control loop delayed system A")
ylabel("Amplitude")
xlabel("time (s)")
stepinfo(ControlSystemA.y, ControlSystemA.tout)


%% Functions

function [phaseShifts, amplitudeRatios, frequency] = generateBode(samples, step, Tsample, Tend, t)
    phaseShifts = [];
    amplitudeRatios = [];
    frequency = [];
    loopEnd = step*samples;
    for omega = 1:step:loopEnd
        u = ones(size(t,1),1);
        u = u.*sin(2*pi*omega.*t);
        systemA = sim("transferfunction12A_harness.slx",t);
    
        [ampRatio, phaseShift, max_input_index, max_output_index] = analyse(omega, Tsample, Tend, u, systemA.y);
        phaseShifts(end+1) = phaseShift;
        amplitudeRatios(end+1) = ampRatio;
        frequency(end+1) = 2*pi*omega;
%         figure(13)
%         plot(t, u, Color= [1,0,0])
%         hold on 
%         xline(max_input_index/Tsample, Color=[1,0,0])
%         plot(systemA.tout, systemA.y, Color=[0,1,0])
%         xline(max_output_index/Tsample, Color=[0,1,0])
%         hold off
    end

end


function [ampRatio, phaseShift, max_input_index, max_output_index] = analyse(freq, samples, duration, input, output)
    % We wait until we are in steady state
    input_length = length(input);
    output_length = length(output);
    input = input(input_length/2:input_length);
    output = output(output_length/2:output_length);

    % We extract the maximums of the input and output
    [max_input, max_input_index] = max(input);
    [max_output, max_output_index] = max(output);
    ampRatio = max_output/max_input;
    max_input_index = max_input_index + input_length/2;
    max_output_index = max_output_index + output_length/2;
    
    % At what time do we reach these maxima?
    t1 = max_input_index/samples;
    t2 = max_output_index/samples;

    cyclePeriod = 1/freq;
    overshoot = (t2-t1)/cyclePeriod;
    deltaT = (t2-t1)-ceil(overshoot)*cyclePeriod;

    
    % We now get the phase shift
    rawPhaseShift = 360*freq*(deltaT);
    phaseShift = -wrapTo360(rawPhaseShift);

end
