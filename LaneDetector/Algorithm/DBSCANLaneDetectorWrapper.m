classdef DBSCANLaneDetectorWrapper < matlab.System
%DBSCANLANEDETECTORWRAPPER  Bird's-eye DBSCAN lane detector for the test bench.
%
%   Drop-in replacement for HelperLaneDetectorWrapper. Builds a bird's-eye
%   view from the camera (using the same geometry as the example's
%   helperLaneDetector), runs Canny+Hough+DBSCAN on the warped image, and
%   outputs the same "LaneSensor" bus the decision logic expects.
%
%   Use it: in LaneMarkerDetector.slx, point the MATLAB System block at
%   DBSCANLaneDetectorWrapper and set "Simulate using -> Interpreted
%   execution". The block mask passes the workspace "camera" struct into
%   the Camera property, so no other wiring changes are needed.
%
%   LaneSensor field mapping (from a parabolicLaneBoundary, Parameters [A B C],
%   y = A*x^2 + B*x + C):  Curvature = 2*A,  HeadingAngle = B,  LateralOffset = C.
%
%   Requires (interpreted execution): Image Processing Toolbox, Statistics
%   and Machine Learning Toolbox (dbscan), Computer Vision / Automated
%   Driving Toolbox (monoCamera, birdsEyeView, parabolicLaneBoundary).

    properties (Nontunable)
        % Display a debug overlay window during normal simulation
        EnableDisplays (1,1) logical = true;
        % Width (m) of the bird's-eye view, and its output image width (px).
        WidthOfBirdsEyeView = 16;
        OutImageWidth       = 250;
    end

    properties
        % Camera sensor parameters (the model passes its "camera" struct here).
        Camera = struct('ImageSize',[768 1024],'PrincipalPoint',[512 384], ...
            'FocalLength',[512 512],'Position',[1.8750 0 1.2000], ...
            'PositionSim3d',[0.5700 0 1.2000],'Rotation',[0 0 0], ...
            'LaneDetectionRanges',[6 30],'DetectionRanges',[6 50], ...
            'MeasurementNoise',diag([6,1,1]));

        % Fallback boundary params [A B C] used when nothing is detected
        DefaultLeftLaneParams  = [0 0  1.925];
        DefaultRightLaneParams = [0 0 -1.925];
    end

    properties (Access = private)
        Sensor
        BEVConfig
        VehicleROI
        DetParams
        LastLeft
        LastRight
        HaveLeft  = false;
        HaveRight = false;
    end

    methods (Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            cam    = obj.Camera;
            intr   = cameraIntrinsics(cam.FocalLength, cam.PrincipalPoint, cam.ImageSize);
            height = cam.Position(3);
            pitch  = cam.Rotation(2);
            obj.Sensor = monoCamera(intr, height, 'Pitch', pitch);

            % Bird's-eye view geometry (matches helperLaneDetector)
            bottomOffset = cam.LaneDetectionRanges(1);
            distAhead    = cam.LaneDetectionRanges(2);
            half         = obj.WidthOfBirdsEyeView/2;
            outView      = [bottomOffset, distAhead, -half, half];
            obj.BEVConfig   = birdsEyeView(obj.Sensor, outView, [NaN obj.OutImageWidth]);
            % obj.VehicleROI  = outView - [-1, 2, -4, 4];   % [7 28 -4 4] by default
            obj.VehicleROI = [7 28 -4 4];

            % DBSCAN parameters for the bird's-eye image
            p = struct();
            p.gaussSigma = 1.0;  p.cannyThresh = [0.1 0.3];
            p.rhoRes = 1;  p.thetaStep = 1;  p.numPeaks = 80;
            p.peakThreshFrac = 0.20;  p.fillGap = 50;  p.minLen = 20;
            p.minAngle = 25;  p.epsilon = 0.1;  p.minPts = 1;  p.roiMask = [];
            obj.DetParams = p;

            obj.LastLeft  = parabolicLaneBoundary(obj.DefaultLeftLaneParams);
            obj.LastRight = parabolicLaneBoundary(obj.DefaultRightLaneParams);
            obj.HaveLeft  = false;
            obj.HaveRight = false;
        end

        %------------------------------------------------------------------
        function lanes = stepImpl(obj, frame)
            % persistent saved
            % if isempty(saved)
            %     imwrite(frame, 'unreal_frame.png');
            %     saved = true;
            % end

            maxX = obj.Camera.LaneDetectionRanges(2);

            [~,~,~,~, leftEgoBoundary, rightEgoBoundary] = detectLanesVehicle( ...
                frame, obj.BEVConfig, obj.DetParams, maxX, obj.VehicleROI);

            if nnz(leftEgoBoundary.Parameters)
                obj.LastLeft = leftEgoBoundary;  obj.HaveLeft = true;
            elseif obj.HaveLeft
                leftEgoBoundary = obj.LastLeft;
            end
            if nnz(rightEgoBoundary.Parameters)
                obj.LastRight = rightEgoBoundary;  obj.HaveRight = true;
            elseif obj.HaveRight
                rightEgoBoundary = obj.LastRight;
            end

            if isempty(coder.target) && obj.EnableDisplays
                obj.displayOverlay(frame, leftEgoBoundary, rightEgoBoundary);
            end

            lanes = obj.packLaneBoundaryDetections(leftEgoBoundary, rightEgoBoundary);
        end
        %------------------------------------------------------------------

        function resetImpl(obj)
            obj.HaveLeft  = false;
            obj.HaveRight = false;
        end
        %------------------------------------------------------------------
        function detections = packLaneBoundaryDetections(obj, leftEgoBoundary, rightEgoBoundary)
            DefaultLanesLeft = struct( ...
                'Curvature',           {single(obj.DefaultLeftLaneParams(1))}, ...
                'CurvatureDerivative', {single(0)}, ...
                'HeadingAngle',        {single(obj.DefaultLeftLaneParams(2))}, ...
                'LateralOffset',       {single(obj.DefaultLeftLaneParams(3))}, ...
                'Strength',            {single(0)}, ...
                'XExtent',             {single([0 0])}, ...
                'BoundaryType',        {LaneBoundaryType.Unmarked});
            DefaultLanesRight = struct( ...
                'Curvature',           {single(obj.DefaultRightLaneParams(1))}, ...
                'CurvatureDerivative', {single(0)}, ...
                'HeadingAngle',        {single(obj.DefaultRightLaneParams(2))}, ...
                'LateralOffset',       {single(obj.DefaultRightLaneParams(3))}, ...
                'Strength',            {single(0)}, ...
                'XExtent',             {single([0 0])}, ...
                'BoundaryType',        {LaneBoundaryType.Unmarked});

            detections = struct('Left', DefaultLanesLeft, 'Right', DefaultLanesRight);

            detections.Left.Curvature(:)     = 2 * leftEgoBoundary.Parameters(1);
            detections.Left.HeadingAngle(:)  = leftEgoBoundary.Parameters(2);
            detections.Left.LateralOffset(:) = leftEgoBoundary.Parameters(3);
            detections.Left.Strength(:)      = leftEgoBoundary.Strength;
            detections.Left.XExtent(:)       = leftEgoBoundary.XExtent;
            detections.Left.BoundaryType(:)  = leftEgoBoundary.BoundaryType;

            detections.Right.Curvature(:)     = 2 * rightEgoBoundary.Parameters(1);
            detections.Right.HeadingAngle(:)  = rightEgoBoundary.Parameters(2);
            detections.Right.LateralOffset(:) = rightEgoBoundary.Parameters(3);
            detections.Right.Strength(:)      = rightEgoBoundary.Strength;
            detections.Right.XExtent(:)       = rightEgoBoundary.XExtent;
            detections.Right.BoundaryType(:)  = rightEgoBoundary.BoundaryType;

            % Shift lateral offset from the camera mount to the vehicle center
            xShift = -obj.Camera.PositionSim3d(1);

            % Lvalid = nnz(leftEgoBoundary.Parameters)  > 0;
            % Rvalid = nnz(rightEgoBoundary.Parameters) > 0;
            % Loff = NaN; Roff = NaN;
            % if Lvalid, Loff = polyval(leftEgoBoundary.Parameters,  xShift); end
            % if Rvalid, Roff = polyval(rightEgoBoundary.Parameters, xShift); end
            % fprintf('Lvalid=%d Loff=%.3f | Rvalid=%d Roff=%.3f\n', Lvalid, Loff, Rvalid, Roff);

            if nnz(leftEgoBoundary.Parameters)
                detections.Left.LateralOffset(:) = polyval(leftEgoBoundary.Parameters, xShift);
                if detections.Left.LateralOffset < 0
                    detections.Left = DefaultLanesLeft;
                end
            else
                detections.Left = DefaultLanesLeft;
            end
            if nnz(rightEgoBoundary.Parameters)
                detections.Right.LateralOffset(:) = polyval(rightEgoBoundary.Parameters, xShift);
                if detections.Right.LateralOffset > 0
                    detections.Right = DefaultLanesRight;
                end
            else
                detections.Right = DefaultLanesRight;
            end
        end

        %------------------------------------------------------------------
        function displayOverlay(obj, frame, lb, rb)
            xv = obj.Camera.LaneDetectionRanges(1):obj.Camera.LaneDetectionRanges(2);
            if ~nnz(lb.Parameters), lb = parabolicLaneBoundary.empty; end
            if ~nnz(rb.Parameters), rb = parabolicLaneBoundary.empty; end
            f = insertLaneBoundary(frame, lb, obj.Sensor, xv, 'Color', 'Red');
            f = insertLaneBoundary(f,     rb, obj.Sensor, xv, 'Color', 'Green');
            persistent player
            if isempty(player)
                player = vision.DeployableVideoPlayer('Name', 'DBSCAN lane detections');
            end
            step(player, f);
        end

        % --- Output specification: a single LaneSensor bus -----------------
        function lanes = getOutputSizeImpl(~),     lanes = 1;            end
        function lanes = getOutputDataTypeImpl(~), lanes = "LaneSensor"; end
        function lanes = isOutputComplexImpl(~),   lanes = false;        end
        function lanes = isOutputFixedSizeImpl(~), lanes = true;         end
    end
end