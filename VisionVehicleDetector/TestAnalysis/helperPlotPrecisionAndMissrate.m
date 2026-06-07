function helperPlotPrecisionAndMissrate(detectionMetrics, detector)
% helperPlotPrecisionAndMissrate A helper function for plotting the vehicle
% detector metrics computed from VisionVehicleDetectorTestBench.slx.
%
% This is a helper function for example purposes and may be removed or
% modified in the future.
%
% The function assumes that the demo outputs the Simulink log, logsout,
% containing the elements to be plotted.

% Copyright 2020-2023 The MathWorks, Inc.

arguments (Repeating)
    detectionMetrics
    detector string
end

newLineDispOffset = 0;

% Recall vs Precision figure
f1 = figure;
f1.Name= 'Recall Vs Precision';
f1.Position = [835 100 720 600];

axPrecVsRecall = axes(f1);

hold(axPrecVsRecall,"on");
grid(axPrecVsRecall,"on");
ylim(axPrecVsRecall,[0 1]);
title(axPrecVsRecall,'Recall Vs Precision ');

ylimit = get(axPrecVsRecall,'ylim');
xlimit = get(axPrecVsRecall,'xlim');
text(axPrecVsRecall,xlimit(1), ylimit(2), sprintf(' \n\n Average Precision'))

xlabel(axPrecVsRecall,'Recall');
ylabel(axPrecVsRecall,'Precision');


% FPPI Vs Missrate figure
f2 = figure;
f2.Name= 'FPPI Vs Missrate';
f2.Position = [835 100 720 600];

axFppiVsMr = axes(f2);

hold(axFppiVsMr,"on");
grid(axFppiVsMr,"on");
ylim(axFppiVsMr,[0 1]);
title(axFppiVsMr,'FPPI Vs Missrate ', 'Color', 'black');

ylimit = get(axFppiVsMr,'ylim');
xlimit = get(axFppiVsMr,'xlim');
text(axFppiVsMr,xlimit(1) , ylimit(2), sprintf(' \n\n Average Missrate'));

xlabel(axFppiVsMr,'False Positives Per Image (FPPI)');
ylabel(axFppiVsMr,'Miss Rate');
    

% Compute the precision and miss rate from  logsout
for i = 1:size(detectionMetrics,2)
    newLineDispOffset = newLineDispOffset + 0.05;

    plot(axPrecVsRecall,detectionMetrics{i}.recall, detectionMetrics{i}.precision);
    text(axPrecVsRecall,xlimit(1), ylimit(2) - newLineDispOffset , sprintf(' \n\n %s = %.1f ', detector{1,i}, detectionMetrics{i}.avgPrecision),'FontSize', 8);

    
    loglog(axFppiVsMr,detectionMetrics{i}.fppi, detectionMetrics{i}.missRate);
    text(axFppiVsMr,xlimit(1) , ylimit(2) - newLineDispOffset , sprintf(' \n\n %s = %.1f', detector{1,i}, detectionMetrics{i}.averageMissrate), 'FontSize', 8);

end

hold(axPrecVsRecall,"off");
hold(axFppiVsMr,"off");


end
