function [leftBoundary, rightBoundary, leftValid, rightValid, lbObj, rbObj] = ...
        detectLanesVehicle(I, birdsEyeConfig, params, maxX, vehicleROI)
%DETECTLANESVEHICLE  Bird's-eye DBSCAN detection -> vehicle-coordinate lanes.
%
%   [leftBoundary, rightBoundary, leftValid, rightValid, lbObj, rbObj] = ...
%       detectLanesVehicle(I, birdsEyeConfig, params, maxX, vehicleROI)
%
%   Warps the perspective camera frame into a bird's-eye view, runs the
%   Canny+Hough+DBSCAN detector on it, projects the detected points into
%   vehicle coordinates, and fits a parabola y = a*x^2 + b*x + c to the
%   left and right boundaries. This is the representation the Highway Lane
%   Following decision logic consumes.
%
%   Inputs
%     I               perspective camera frame (RGB/grayscale).
%     birdsEyeConfig  birdsEyeView object (built once in the wrapper).
%     params          struct for detectLanesDBSCAN ([] = defaults).
%     maxX            max longitudinal look-ahead (m), default 40.
%     vehicleROI      [xmin xmax ymin ymax] in vehicle coords (m). Restricts
%                     detection to the road region; converted to a BEV mask.
%
%   Outputs
%     leftBoundary/rightBoundary  1x7 [Curvature, CurvatureDerivative,
%                     HeadingAngle, LateralOffset, Strength, Xmin, Xmax]
%     leftValid/rightValid        logical detection flags
%     lbObj/rbObj                 parabolicLaneBoundary objects
%
%   Vehicle coords: X forward, Y left-positive. Left boundary has Y>0.

    if nargin < 3, params = []; end
    if nargin < 4 || isempty(maxX), maxX = 40; end
    if nargin < 5, vehicleROI = []; end

    % --- Warp perspective frame to bird's-eye view ------------------------
    bev = transformImage(birdsEyeConfig, I);

    % --- Restrict to the vehicle ROI (rectangle -> BEV pixel mask) --------
    if ~isempty(vehicleROI)
        if isempty(params), params = struct(); end
        params.roiMask = bevROIMask(birdsEyeConfig, vehicleROI, ...
                                    size(bev,1), size(bev,2));
    end

    % --- DBSCAN detection on the bird's-eye image -------------------------
    lanes = detectLanesDBSCAN(bev, params);

    % --- Project detected points into vehicle coordinates -----------------
    leftXY  = zeros(0,2);
    rightXY = zeros(0,2);
    for i = 1:numel(lanes)
        veh = imageToVehicle(birdsEyeConfig, lanes(i).points);  % [X Y] (m)
        ok  = isfinite(veh(:,1)) & isfinite(veh(:,2)) & ...
              veh(:,1) > 0 & veh(:,1) < maxX;
        veh = veh(ok,:);
        if size(veh,1) < 3, continue; end
        if mean(veh(:,2)) >= 0
            leftXY  = [leftXY;  veh];           %#ok<AGROW>
        else
            rightXY = [rightXY; veh];           %#ok<AGROW>
        end
    end

    % --- Fit a parabola per side ------------------------------------------
    [leftBoundary,  leftValid,  lbObj] = fitVehicleBoundary(leftXY,  maxX);
    [rightBoundary, rightValid, rbObj] = fitVehicleBoundary(rightXY, maxX);
end

% ========================================================================
function mask = bevROIMask(bec, vROI, H, W)
%BEVROIMASK  Convert a vehicle-coordinate ROI rectangle to a BEV pixel mask.
%   vROI = [xmin xmax ymin ymax] (m). Corners are projected with
%   vehicleToImage and rasterized with poly2mask.
    c = [vehicleToImage(bec, [vROI(1) vROI(3)]);
         vehicleToImage(bec, [vROI(1) vROI(4)]);
         vehicleToImage(bec, [vROI(2) vROI(4)]);
         vehicleToImage(bec, [vROI(2) vROI(3)])];   % each [col row]
    mask = poly2mask(c(:,1), c(:,2), H, W);
end

% ========================================================================
function [vec, valid, obj] = fitVehicleBoundary(xy, maxX)
%FITVEHICLEBOUNDARY  Least-squares parabolic fit in vehicle coordinates.
    vec   = zeros(1,7);
    valid = false;
    obj   = parabolicLaneBoundary([0 0 0]);

    if size(xy,1) >= 3
        x = xy(:,1);  y = xy(:,2);
        p = polyfit(x, y, 2);              % y = p(1)*x^2 + p(2)*x + p(3)
        a = p(1);  b = p(2);  c = p(3);

        curvature           = 2*a;
        curvatureDerivative = 0;
        headingAngle        = atan(b);
        lateralOffset       = c;
        xmin = min(x);  xmax = max(x);
        strength = min(1, (xmax - xmin)/maxX);

        vec = [curvature, curvatureDerivative, headingAngle, ...
               lateralOffset, strength, xmin, xmax];

        obj              = parabolicLaneBoundary(p);
        obj.Strength     = strength;
        obj.XExtent      = [xmin xmax];
        obj.BoundaryType = LaneBoundaryType.Solid;
        valid            = true;
    end
end