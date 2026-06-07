classdef HelperACFVehicleDetector < matlab.System
%HelperACFVehicleDetector Provides vehicle detections on image frame.
    % HelperACFVehicleDetector estimates vehicle positions on the 
    % image frame provided by monoCamera sensor.
    % NOTE: The name of this System Object and it's functionality may 
    % change without notice in a future release, 
    % or the System Object itself may be removed.
    
    % Copyright 2020-2023 The MathWorks, Inc.

    properties(Nontunable)
        % Camera Sensor parameters
        Camera = struct('ImageSize',[768 1024],'PrincipalPoint',...
            [512 384],'FocalLength',[512 512],'Position',[1.8750 0 1.2000],...
            'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0],...
            'DetectionRanges',[6 30],'MeasurementNoise',diag([6,1,1]));
        
        % Max Detections
        MaxDetections = 15;

        % ACF Detector Model
        ACFModel = struct

    end
    
    properties (SetAccess='private', GetAccess='private', Hidden)
        
        % MonoDetector holds acf detector object returned by the
        % configureDetectorMonoCamera function.
        MonoDetector;
        
        % Detections that have scores less than this threshold value are
        % removed
        VehicleDetectionThreshold;
        
        % Threshold to increse the speed at the cost of accuracy for ACF
        % detector.
        ClassificationAccuracyThreshold;
    end
    
    methods
        function obj = HelperACFVehicleDetector(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %------------------------------------------------------------------
        % System object methods for Simulink integration
        %------------------------------------------------------------------
        function setupImpl(obj)
            % Camera setup
            %-------------
            camera = obj.Camera;
            
            camIntrinsics = cameraIntrinsics(camera.FocalLength, ...
                camera.PrincipalPoint, camera.ImageSize);
            
            sensor = monoCamera(camIntrinsics, camera.Position(3),...
                'Pitch', camera.Rotation(2));
            
            % Define the threshold for filtering detections based on
            % score.
            obj.VehicleDetectionThreshold = 20;
            
            % Define accuracy threshold for ACF detector.
            obj.ClassificationAccuracyThreshold = -1;
            
            % The width of common vehicles is between 1.5 to 2.5 meters.
            vehicleWidth = [1.5, 2.5];
            
            % Detect objects in monocular camera using ACF features
            obj.MonoDetector = acfObjectDetectorMonoCamera(...
                obj.ACFModel.Classifier, obj.ACFModel.TrainingOptions,...
                sensor, vehicleWidth);
        end
        
        function [bboxes, scores] = stepImpl(obj,frame)

           % Define the ROI for detection
            roi = [1 1 obj.Camera.ImageSize(2) round(obj.Camera.ImageSize(1)*0.678)];

            % Detect the vehicles.
            [bboxes, scores] = detect(obj.MonoDetector,frame,roi,'Threshold',obj.ClassificationAccuracyThreshold);
            
            % Remove detections with low classification scores
            if ~isempty(scores)
                ind = scores >= obj.VehicleDetectionThreshold;
                bboxes = bboxes(ind, :);
                scores = scores(ind, :);
            end  
        end
        
        function [bboxes, scores] = getOutputSizeImpl(obj) 
            % Return size for each output port
            bboxes = [obj.MaxDetections 4];
            scores = [obj.MaxDetections 1];
        end
        
        function [bboxes, scores] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            bboxes = "double";
            scores = "double";
        end
        
        function [bboxes, scores] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            bboxes = false;
            scores = false;
        end
        
        function [bboxes, scores] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            bboxes = false;
            scores = false;
        end
    end
    
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperACFVehicleDetector",...
                "Text",...
                "Detects vehicles from a monocamera image.");
        
        end
        
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end
end
