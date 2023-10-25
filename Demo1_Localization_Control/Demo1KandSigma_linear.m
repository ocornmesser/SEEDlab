%% Runmotorsim.m LINEAR
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Description
% This is the Matlab code for 2A.
% This code is ultilized from the tutorial handout.
%In order to get the data appropiately, make a blank array
% in the command prompt data =[]
% Next open up putty and copy and paste the outputs into excel,
% use the data wizard and format the outputs appropiately
% click on data in matlab and copy and paste the contents in. 
%% Loading the data in from the steps above
 % loading in the data to use down below
 
filename = 'myData(1.1).xlsx';
newDataDemo1 = xlsread(filename); 

%% Define motor parameters
K=0.06; % DC gain [rad/Vs] we don't need the /2 FIX THIS.
sigma= 8.0; % time constant reciprocal [1/s]
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('Demo1KandSigma') % this code opens the feedback loop in simulink
%
% run the simulation
%

out=sim('Demo1KandSigma'); % this is taking the outputs (outs) from the simulation to graph
%% A Plot of the results
% this code simply graphs the simulated and experimental outputs on top of 
% one another in order for us to see and compare our actual experiment from
% the simulation of out motor.
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(newDataDemo1(:,1),newDataDemo1(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'--','linewidth',2)
hold on
plot(newDataDemo1(:,1),newDataDemo1(:,3),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Velocity m/s') % double check what this is angular or regular


