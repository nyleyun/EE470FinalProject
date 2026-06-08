function [laneModels, debug] = detectLanesDBSCAN(I, params)
%DETECTLANESDBSCAN  Lane-line discrimination with Canny + Hough + DBSCAN.
%
%   [laneModels, debug] = detectLanesDBSCAN(I, params)
%
%   Pipeline (unchanged methodology -- now intended to run on a bird's-eye
%   view image, where lane lines are straight, parallel, and vertical):
%     1. Grayscale + Gaussian smoothing
%     2. Canny edge detection
%     3. Region-of-interest mask (params.roiMask if supplied, else a
%        trapezoid from params.roi)
%     4. Hough transform -> line segments (houghlines)
%     5. Each segment -> feature vector [x-intercept@bottom, slope]
%     6. DBSCAN groups segments belonging to the same lane marker
%     7. Robust line fit per cluster -> lane boundary model
%
%   Inputs
%     I       road image (uint8/double), perspective OR bird's-eye view.
%     params  struct of tunable parameters. Any missing fields are filled
%             from defaultLaneParams(). Supply params.roiMask (a logical
%             mask the size of I) to use an arbitrary ROI -- this is how the
%             bird's-eye pipeline passes the rectangular vehicle ROI.
%
%   Outputs
%     laneModels  struct array, one element per detected boundary:
%                   .coeffs  [m b]  with  x = m*y + b   (image pixel coords)
%                   .side    'left' | 'right'
%                   .points  Nx2 sampled [x y] = [col row] points
%     debug       struct of intermediate results for visualization.
%
%   Requires: Image Processing Toolbox, Statistics and Machine Learning
%   Toolbox (for dbscan).

    % ---- Merge supplied params over defaults (fill any missing fields) ----
    defaults = defaultLaneParams();
    if nargin < 2 || isempty(params)
        params = defaults;
    else
        fn = fieldnames(defaults);
        for i = 1:numel(fn)
            if ~isfield(params, fn{i}) || isempty(params.(fn{i}))
                params.(fn{i}) = defaults.(fn{i});
            end
        end
    end

    % ---- 1. Pre-processing -------------------------------------------------
    if size(I,3) == 3
        gray = rgb2gray(I);
    else
        gray = I;
    end
    gray = im2double(gray);
    gray = imgaussfilt(gray, params.gaussSigma);

    [H_img, W_img] = size(gray);

    % ---- 2. Canny edges ----------------------------------------------------
    edges = edge(gray, 'canny', params.cannyThresh);

    % ---- 3. Region of interest --------------------------------------------
    if ~isempty(params.roiMask)
        roiMask = logical(params.roiMask);
    else
        roiMask = roiTrapezoid([H_img W_img], params.roi);
    end
    edges = edges & roiMask;

    % ---- 4. Hough transform + line segments --------------------------------
    [Hacc, theta, rho] = hough(edges, 'RhoResolution', params.rhoRes, ...
                                      'Theta', -89:params.thetaStep:89);
    peaks = houghpeaks(Hacc, params.numPeaks, ...
                       'Threshold', ceil(params.peakThreshFrac*max(Hacc(:))));
    segs  = houghlines(edges, theta, rho, peaks, ...
                       'FillGap', params.fillGap, 'MinLength', params.minLen);

    % ---- 5. Feature vector per segment -------------------------------------
    % x = m*y + b parametrization (handles vertical lane lines).
    % Feature = [normalized x-intercept at image bottom, slope m].
    feats = [];
    keep  = struct('p1',{},'p2',{});
    for k = 1:numel(segs)
        p1 = segs(k).point1;  p2 = segs(k).point2;     % [x y] = [col row]
        dx = p2(1)-p1(1);     dy = p2(2)-p1(2);
        ang = atan2d(dy, dx);
        % Reject near-horizontal segments (not lane lines)
        if abs(ang) < params.minAngle || abs(ang) > (180 - params.minAngle)
            continue;
        end
        m = dx/dy;
        b = p1(1) - m*p1(2);
        xbot = m*H_img + b;
        feats(end+1,:) = [xbot/W_img, m];                 %#ok<AGROW>
        keep(end+1)    = struct('p1',p1,'p2',p2);         %#ok<AGROW>
    end

    laneModels = struct('coeffs',{},'side',{},'points',{});
    debug = struct('edges',edges,'segments',segs,'feats',feats, ...
                   'roi',roiMask,'clusterIdx',[]);

    if isempty(feats)
        return;
    end

    % ---- 6. DBSCAN clustering ----------------------------------------------
    % Standardize features, but do NOT amplify a near-constant dimension
    % (in the bird's-eye view all slopes are ~0, so its std is tiny).
    mu = mean(feats,1);
    sg = std(feats,0,1);
    sg(sg < 1e-3) = 1;
    F = (feats - mu) ./ sg;
    clusterIdx = dbscan(F, params.epsilon, params.minPts);
    debug.clusterIdx = clusterIdx;

    % ---- 7. Fit one boundary per cluster -----------------------------------
    ids = unique(clusterIdx(clusterIdx > 0));
    for c = ids'
        idxList = find(clusterIdx == c);
        pts = zeros(2*numel(idxList), 2);
        for j = 1:numel(idxList)
            pts(2*j-1,:) = keep(idxList(j)).p1;
            pts(2*j,  :) = keep(idxList(j)).p2;
        end

        Y = pts(:,2);  X = pts(:,1);
        mb = [Y ones(size(Y))] \ X;          % x = m*y + b
        m = mb(1);  b = mb(2);

        yy = linspace(min(Y), H_img, 20)';
        xx = m*yy + b;

        xbot = m*H_img + b;
        if xbot < W_img/2, side = 'left'; else, side = 'right'; end

        laneModels(end+1) = struct('coeffs',[m b], 'side',side, ...
                                   'points',[xx yy]);        %#ok<AGROW>
    end
end

% ========================================================================
function p = defaultLaneParams()
%DEFAULTLANEPARAMS  Starting parameters tuned for a bird's-eye-view image.
    p.gaussSigma     = 1.0;
    p.cannyThresh    = [0.08 0.25];   % [low high]; BEV stripes are bright
    p.rhoRes         = 1;
    p.thetaStep      = 1;
    p.numPeaks       = 80;
    p.peakThreshFrac = 0.20;
    p.fillGap        = 50;            % bridge dashed-marker gaps in BEV
    p.minLen         = 20;
    p.minAngle       = 25;            % reject near-horizontal segments
    p.epsilon        = 0.35;          % DBSCAN neighbourhood (standardized)
    p.minPts         = 3;             % DBSCAN minimum cluster size
    p.roiMask        = [];            % supply a logical mask to override ROI
    % Fallback trapezoid (used only if roiMask is empty)
    p.roi.ty  = 0.0;
    p.roi.tlx = 0.05;  p.roi.trx = 0.95;
    p.roi.blx = 0.05;  p.roi.brx = 0.95;
end

% ========================================================================
function mask = roiTrapezoid(sz, roi)
%ROITRAPEZOID  Binary trapezoidal mask (fallback when no roiMask supplied).
    H = sz(1);  W = sz(2);
    xs = [roi.blx*W, roi.tlx*W, roi.trx*W, roi.brx*W];
    ys = [H,         roi.ty*H,  roi.ty*H,  H];
    mask = poly2mask(xs, ys, H, W);
end