classdef HelperCastDetectionsToSingle < matlab.System
%HelperCastDetectionsToSingle Converts datatype of the given bus elements
% from double precision to single precision.
%
% This class is for example purpose only. It may be removed or changed
% in the future.

% Copyright 2022 The MathWorks, Inc.

    properties (Nontunable)
        % Name of output bus
        OutputBusName = 'BusDetections';
    end

    methods (Access = protected)
        function Detections = stepImpl(obj, Detections)
                Detections = castBusToSingle(Detections);
                Detections.NumDetections = int32(Detections.NumDetections);
                for i = 1:numel(Detections.Detections)
                    Detections.Detections(i).SensorIndex = uint32(Detections.Detections(i).SensorIndex);
                    Detections.Detections(i).ObjectClassID = uint32(Detections.Detections(i).ObjectClassID);
                end
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = "Cast";
        end

        function name = getInputNamesImpl(obj)
            % Return input port names for System block
            name = ['Detections',newline,'(double)'];
        end

        function name = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = ['Detections',newline,'(single)'];
        end
    end

    methods (Access = protected)
        function out = getOutputSizeImpl(obj)
            out = [1 1];
        end

        function out = getOutputDataTypeImpl(obj)
            out = obj.OutputBusName;
        end

        function out = isOutputComplexImpl(obj)
            out = false;
        end


    end
end

function out = castBusToSingle(in)
% Input(in) is a struct/bus and output(out) is a single-precision struct
if ~isscalar(in)
    sample = castBusToSingle(in(1));
    out = repmat(sample,size(in));
    for i = 1:numel(in)
        out(i) = castBusToSingle(in(i));
    end
    return;
end

fNames = fieldnames(in);
for i = 1:numel(fNames)
    if isstruct(in.(fNames{i}))
        out.(fNames{i}) = castBusToSingle(in.(fNames{i}));
    elseif isa(in.(fNames{i}),'double')
        out.(fNames{i}) = single(in.(fNames{i}));
    else
        out.(fNames{i}) = in.(fNames{i});
    end
end
end