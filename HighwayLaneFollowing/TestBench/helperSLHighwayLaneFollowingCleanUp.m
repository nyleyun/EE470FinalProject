% Clean up script for the Highway Lane Following Example
%
% This script cleans up the Highway Lane Following example model. It is
% triggered by the CloseFcn callback.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2019-2023 The MathWorks, Inc.

clearBuses({ ...
    'BusActors1',...
    'BusDetectionConcatenation1',...
    'BusDetectionConcatenation1Detections',...
    'BusDetectionConcatenation1DetectionsMeasurementParameters',...
    'BusLaneBoundaries1',...
    'BusLaneBoundaries1LaneBoundaries',...
    'BusRadar',...
    'BusRadarDetections',...
    'BusRadarDetectionsMeasurementParameters',...
    'BusRadarDetectionsObjectAttributes',...
    'BusVehiclePose',...
    'BusVision',...
    'BusVisionDetections',...
    'BusVisionDetectionsMeasurementParameters',...
    'BusVisionDetectionsObjectAttributes',...
    'BusLaneCenter',...
    'BusTrackerJPDA',...
    'BusTrackerJPDATracks',...
    'BusVisionDetectionsInfo',...
    'BusVisionInfo'});

clear actorProfiles
clear actorDimensions
clear alpha
clear Cf
clear Cr
clear cutOffDistance
clear Iz
clear LaneSensor
clear LaneSensorBoundaries
clear PredictionHorizon
clear ControlHorizon
clear Ts
clear assessment
clear assigThresh
clear camera
clear default_spacing
clear vehicleDetectionRange
clear egoVehDyn
clear lf
clear lr
clear m
clear max_ac
clear max_steer
clear min_ac
clear min_steer
clear numSensors
clear numTracks
clear posSelector
clear radar
clear scenario
clear scenarioFcnName
clear switchingPenalty
clear tau
clear time_gap
clear v0_ego
clear v_set
clear vehSim3D
clear max_dc
clear tau2
clear driver_decel
clear FB_decel
clear headwayOffset
clear PB1_decel
clear PB2_decel
clear timeMargin
clear timeToReact
clear Default_decel
clear detectorVariant
clear Epsilon
clear LaneWidth
clear M
clear MinNumPoints 
clear N
clear P 
clear R 
clear stopVelThreshold
clear TimeFactor
clear egoActorID
clear TimeDataType
clear order
clear acfDetectorStruct


function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end
