function hFigure = helperPlotFVSFDiffSignals(diffResult)
% helperPlotFVSFDiffSignals Plots the signal differences that are
% computed from two simulation runs (normal and PIL simulations) of
% ForwardVehicleSensorFusionTestBench.slx
%
% This is a helper function for example purposes and may be removed or
% modified in the future.
%
% The function assumes that the demo outputs the Simulink log, logsout,
% containing the following elements to be plotted.

% Copyright 2022 The MathWorks, Inc.

%% Get the diff data from normal and PIL simulation

% GOSPA
GOSPA = diffResult.getResultsByName('gospa');                                 

% falseTrack
falseTrack = diffResult.getResultsByName('false_tracks_error');                        

% localization
localization = diffResult.getResultsByName('localization_error');

% missTarget
missTarget = diffResult.getResultsByName('miss_target_error');

% simulation time
tmax = GOSPA.Diff.Time(end);                                              

%% Plot the results
hFigure = figure('Name','Forward Vehicle Sensor Fusion Metrics Difference','position',[100 100 720 600]);

%% GOSPA metric
subplot(4,1,1)
plot(GOSPA.Diff.time, GOSPA.Diff.Data(:,:)','b');
grid on
xlim([0,tmax])
ylim([-0.01 0.01]);
title('GOSPA')
xlabel('time (sec)')
ylabel('diff value')

%% falseTrack 
subplot(4,1,2)
plot(falseTrack.Diff.time,falseTrack.Diff.Data(:,:)','b')
grid on
xlim([0,tmax])
ylim([-0.01 0.01]);
title('falseTrack')
xlabel('time (sec)')
ylabel('diff value')

%% localization
subplot(4,1,3)
plot(localization.Diff.time,localization.Diff.Data(:,:)','b')
grid on
xlim([0,tmax])
ylim([-0.01 0.01]);
title('localization')
xlabel('time (sec)')
ylabel('diff value')

%% missTarget
subplot(4,1,4)
plot(missTarget.Diff.time,missTarget.Diff.Data(:,:)','b')
grid on
xlim([0,tmax])
ylim([-0.01 0.01]);
title('missTarget')
xlabel('time (sec)')
ylabel('diff value')
end
