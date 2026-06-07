function filter = helperInitFilter(detection)
%Helper function to update the process noise and state covariance of the
%initialized filter from initcvekf.

% Copyright 2021-2022 The MathWorks, Inc.

%#codegen

% initialize filter with initcvekf
filter = initcvekf(detection);

% update processnoise
filter.ProcessNoise = cast(blkdiag(25,5,1e-2),'like',detection.Measurement);

% update statecovariance
filter.StateCovariance = cast(blkdiag(filter.StateCovariance(1:4,1:4),1e-2*eye(2)),'like',detection.Measurement);
end