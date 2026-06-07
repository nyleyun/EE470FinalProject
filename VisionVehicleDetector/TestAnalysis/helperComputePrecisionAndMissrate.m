function detectionMetrics = helperComputePrecisionAndMissrate(logsout)
% helperComputePrecisionAndMissrate A helper function for computing the
% post simulation metrics using logged data from
% VisionVehicleDetectorTestBench.slx.
%
% This is a helper function for example purposes and may be removed or
% modified in the future.
%
% The function assumes that the demo outputs the Simulink log, logsout,
% containing the elements that are needed to compute post simulation
% metrics.

% Copyright 2021-2023 The MathWorks, Inc.
detections = logsout.get('detections');
truth = logsout.get('vehicle_boxes_truth');

tablePredicted = table({[]},{categorical([])},{[]},'VariableNames',{'Boxes','Labels','Scores'});
tableTruth     = table({[]},{categorical([])},'VariableNames',{'Boxes','Labels'});
tableIdx = 1;


for i = 1:length(truth.Values.Time)
    % Detection results for current time
    numDet        = detections.Values.NumDetections.Data(i);
    
    currDetBoxes  = [zeros(0,4)];
    currDetScores = [zeros(0,1)];
    currDetLables = repmat(categorical("car"),[numDet,1]);
    
    for j = 1:numDet
        currDetBoxes  = [currDetBoxes;detections.Values.Detections(j).Boxes.Data(:,:,i)];
        currDetScores = [currDetScores;detections.Values.Detections(j).Scores.Data(:,:,i)];        
    end
    
    tablePredicted(tableIdx,:) = table({currDetBoxes},{currDetLables},{currDetScores},'VariableNames', {'Boxes', 'Labels', 'Scores'});
    
    % Groundtruth for current time
    groundTruthBoxes = truth.Values.Data(:,:,i);
    groundTruthBoxes(groundTruthBoxes(:,4)<=0,:) = [];
    groundTruthLabels = repmat(categorical("car"),size(groundTruthBoxes,1),1);
    tableTruth(tableIdx,:) = table({groundTruthBoxes},{groundTruthLabels},'VariableNames', {'Boxes', 'Labels'});

    tableIdx = tableIdx+1;
end



% Evaluate the detections against the truth
metrics = evaluateObjectDetection(tablePredicted, tableTruth,AdditionalMetrics="LAMR",Verbose=false);

detectionMetrics.avgPrecision    = metrics.ClassMetrics.AP{1};
detectionMetrics.recall          = metrics.ClassMetrics.Recall{1};
detectionMetrics.precision       = metrics.ClassMetrics.Precision{1}; 
detectionMetrics.averageMissrate = metrics.ClassMetrics.LAMR{1};
detectionMetrics.fppi            = metrics.ClassMetrics.FPPI{1};
detectionMetrics.missRate        = metrics.ClassMetrics.MR{1};

end