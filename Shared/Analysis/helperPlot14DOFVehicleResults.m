function [hFigLongResults, hFigLatResults] = helperPlot14DOFVehicleResults(logsout, scenarioFcnName)
% helperPlot14DOFVehicleResults A helper function for plotting the
% results of the 14DOF vehicle model.
%
% This is a helper function for example purposes and may be removed or
% modified in the future.
%
% The function assumes that the simulink model outputs the Simulink log,
% logsout, containing several signals.

% Copyright 2021 The MathWorks, Inc.

if nargin < 2
    scenarioFcnName = '';
end


%% Get the data from simulation
accel = logsout.getElement('accel_cmd'); % input acceleration 
decel = logsout.getElement('decel_cmd'); % input deceleration
ego_velocity = logsout.getElement('ego_velocity'); % output velocity of ego car
longitudinal_acceleration = logsout.getElement('<ax>');  % outpt longitudinal acceleration of ego car
brake_pressure = logsout.getElement('<BrkPrs>');   % output brake presure


steer = logsout.getElement('steer_cmd'); % input steer cmd
yawRate = logsout.getElement('yaw_rate'); % ego yaw rate
lateral_acceleration = logsout.getElement('<ay>');  % lateral acceleration of ego car
gear = logsout.getElement('gear_cmd'); % input gear command

%% Plot the results
hFigLongResults = figure('position',[100 100 720 600]);
set(gcf,'Numbertitle','off');
set(gcf,'Name','AEB With High Fidelity Vehicle Simulation Result');


% acceleration and deceleration input commands
subplot(4,1,1);
hold on;
plot(accel.Values.time,accel.Values.Data,'b','LineWidth',2);
plot(decel.Values.time,decel.Values.Data, 'm' , 'LineWidth', 2);
xlim([0, accel.Values.time(end)]);
legend({'Acceleration','Deceleration'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
msg = sprintf('Longitudinal Results - %s\n\nEgo Car Acceleration and Deceleration Inputs', scenarioFcnName);
title(msg,'Interpreter','none')
ylabel('m/s^{2}');
grid on;


% ego velocity
subplot(4,1,2);
hold on;
plot(ego_velocity.Values.time,ego_velocity.Values.Data,'b','LineWidth',2);
xlim([0, ego_velocity.Values.time(end)]);
legend({'Ego Velocity'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
title('Ego Car Velocity');
ylabel('m/s');
grid on;

% longitudinal acceleration output
subplot(4,1,3);
hold on;
plot(longitudinal_acceleration.Values.time,longitudinal_acceleration.Values.Data,'m','LineWidth',2);

legend({'Longitudinal Acceleration'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
title('Ego Car Longitudinal Acceleration');
xlim([0, longitudinal_acceleration.Values.time(end)]);
ylabel('g');
grid on;

% brake pressure
subplot(4,1,4);
hold on;
plot(brake_pressure.Values.time, brake_pressure.Values.Data(:,1),'LineWidth', 2);
plot(brake_pressure.Values.time, brake_pressure.Values.Data(:,2),'LineWidth', 2);
plot(brake_pressure.Values.time, brake_pressure.Values.Data(:,3),'LineWidth', 2);
plot(brake_pressure.Values.time, brake_pressure.Values.Data(:,4),'LineWidth', 2);
legend({'Brk Prs(1)', 'Brk Prs(2)', 'Brk Prs(3)', 'Brk Prs(4)'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
ylim([0, inf]);
xlim([0, brake_pressure.Values.time(end)]);
title('Brake Pressure')
ylabel('pascal')
xlabel('time (sec)')
grid on;


hold off;
hFigLatResults = figure('position',[100 100 720 600]);
set(gcf,'Numbertitle','off');
set(gcf,'Name','AEB With High Fidelity Vehicle Simulation Result');


% steer input command
subplot(4,1,1);
hold on;
plot(steer.Values.time,steer.Values.Data,'b','LineWidth',2);
xlim([0, steer.Values.time(end)]);
legend({'Steering Command'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
msg = sprintf('Lateral Results - %s\n\nEgo Car Steering Input', scenarioFcnName);
title(msg,'Interpreter','none')
ylabel('deg');
grid on;

% yaw rate
subplot(4,1,2);
hold on;
plot(yawRate.Values.time,yawRate.Values.Data,'b','LineWidth',2);
xlim([0, yawRate.Values.time(end)]);
legend({'Ego Yaw Rate'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
title('Ego Yaw Rate');
ylabel('m/s');
grid on;

% lateral acceleration output
subplot(4,1,3);
hold on;
plot(lateral_acceleration.Values.time,lateral_acceleration.Values.Data,'m','LineWidth',2);
xlim([0, lateral_acceleration.Values.time(end)]);
legend({'Lateral Acceleration'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
title('Ego Car Lateral Acceleration');
ylabel('g');
grid on;

% Gear command
subplot(4,1,4);
hold on;
plot(gear.Values.time, gear.Values.Data,'LineWidth', 2);
legend({'Gear Command'},...
    'Location','best',...
    'FontName','Arial',...    
    'FontSize',6);
ylim([0, inf]);
xlim([0, gear.Values.time(end)]);
xlabel('time (sec)');
title('Gear')
grid on;