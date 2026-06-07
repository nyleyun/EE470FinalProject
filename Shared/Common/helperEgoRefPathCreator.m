function egoRefPath = helperEgoRefPathCreator(scenario, egoActorID)
% helperEgoRefPathCreator creates reference path for ego vehicle from the
% driving scenario.
%
% Inputs:
%
% scenario:     driving scenario object.
% egoActorID:   Actor ID of the ego vehicle in the scenario.
%
% Outputs:
%  egoRefPath: ego reference path.
%
% This is a helper function for example purposes and may be removed or
% modified in the future.

% Copyright 2021-2023 The MathWorks, Inc.

restart(scenario);
rec = record(scenario); % Run driving scenario and record actor states
num = length(rec);

x = zeros(num,1);
y = zeros(num,1);
theta = zeros(num,1);
speed = zeros(num,1);
kappa = zeros(num,1);

for i = 1:num
    time(i)  = rec(i).SimulationTime;
    x(i)     = rec(i).ActorPoses(egoActorID).Position(1);
    y(i)     = rec(i).ActorPoses(egoActorID).Position(2);
    theta(i) = deg2rad(rec(i).ActorPoses(egoActorID).Yaw);
    speed(i) = norm(rec(i).ActorPoses(egoActorID).Velocity(1:2));
    if(speed(i) > 0)
        kappa(i) = deg2rad(rec(i).ActorPoses(egoActorID).AngularVelocity(3))/speed(i);
    end
end

% Remove duplicates in the position array.
[x_y, indices] = unique([x,y], 'rows', 'stable');

% Calculate interpolate sample points based on distance
interpDistance = 0.1; % Interpolation distance in meters
cumDistance = [0, cumsum(vecnorm(diff(x_y),2,2))'];
interpNumTrajPoints = round(cumDistance(end)/interpDistance);
cumDistanceResample = linspace(0, cumDistance(end), interpNumTrajPoints);

egoRefPath.time = zeros(interpNumTrajPoints,1);
egoRefPath.x = x_y(:,1);
egoRefPath.y = x_y(:,2);

% Interpolate the postion, theta, kappa and speed.
egoRefPath.x = interp1(cumDistance,egoRefPath.x',cumDistanceResample);
egoRefPath.y = interp1(cumDistance,egoRefPath.y',cumDistanceResample);
egoRefPath.theta = interp1(cumDistance,theta(indices),cumDistanceResample);
egoRefPath.kappa = interp1(cumDistance,kappa(indices),cumDistanceResample)';
egoRefPath.speed = interp1(cumDistance,speed(indices),cumDistanceResample)';
egoRefPath.arcLength = cumDistanceResample';

% Update the new number of points after interpolation.
egoRefPath.numPoints = size(egoRefPath.x',1);

egoRefPath.x = egoRefPath.x';
egoRefPath.y = egoRefPath.y';
egoRefPath.theta = egoRefPath.theta';
restart(scenario);
end