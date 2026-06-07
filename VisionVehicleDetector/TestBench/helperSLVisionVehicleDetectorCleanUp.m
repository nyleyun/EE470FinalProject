% Clean up script for the Generate Code For Vision Vehicle Detector Example
%
% This script cleans up the Vision Vehicle Detector example model. It is
% triggered by the CloseFcn callback.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2020-2022 The MathWorks, Inc.

clearBuses({'BusActors1',...
    'BusActors1Actors',...
    'BusVision',...
    'BusVisionDetections',...
    'BusVisionDetectionsMeasurementParameters',...
    'BusVisionDetectionsObjectAttributes',...
    'BusVisionInfo',...
    'BusVisionDetectionsInfo'});

clear camera;
clear detectorVariant;
clear dimensions;
clear scenario;
clear Ts;
clear vehSim3D;
clear acfDetectorStruct


function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end