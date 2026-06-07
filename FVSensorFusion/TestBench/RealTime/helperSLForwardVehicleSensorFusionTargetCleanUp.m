% Clean up script for the Automate Real-Time Testing for Forward Vehicle
% Sensor Fusion Example
%
% This script cleans up the RTForwardVehicleSensorFusionTarget.slx model.
% It is triggered by the CloseFcn callback.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2021-2022 The MathWorks, Inc.

clearBuses({...
    'BusRadar',...
    'BusSimulation3DRadarTruthSensor1Detections',...
    'BusSimulation3DRadarTruthSensor1DetectionsMeasurementParameters',...
    'BusSimulation3DRadarTruthSensor1DetectionsObjectAttributes',...
    'BusTrackerJPDA',...
    'BusTrackerJPDATracks',...
    'BusVision',...
    'BusVisionDetections',...
    'BusVisionDetectionsMeasurementParameters',...
    'BusVisionDetectionsObjectAttributes'});

clear assigThresh
clear Epsilon
clear M
clear MinNumPoints
clear N
clear numSensors
clear numTracks
clear P
clear R
clear Ts
clear RTConfig
clear maxNumRadarDets
clear maxNumTracks
clear maxNumVisionDets
clear TimeDataType

function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end