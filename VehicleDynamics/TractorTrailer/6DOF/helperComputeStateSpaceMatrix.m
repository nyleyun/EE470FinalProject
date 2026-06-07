function [A, B, C] =  helperComputeStateSpaceMatrix(Vx)
%helperComputeStateSpaceMatrix function computes the state space matrix A,B
% and C for the linearized tractor-trailer plant model. The A,B and C
% matrix define an approximate model of tractor-trailer which is used as
% the predictive model by the Lane Keep Assist Block.
%
% This helper script for example purposes and may be removed or modified in
% the future.

% Copyright 2022 The MathWorks, Inc.

m1 = 5500;                 % [kg] Mass of Tractor
I1 = 31488.569009314142;   % [kg m^2] Yaw Moment of Inertia of Tractor
m2 = 7500;                 % [kg] Trailer Mass
I2 = 26240.474174428451;   % [kg m^2] Yaw Moment of Inertia of Trailer

% Approximate model dimensions in [meters]
l1 = 2.3;      
l2 = 0.89;     
l3 = 1.41;     
l4 = 3.19;     
l5 = 4.51;     
l6 = 1.595;    
l7 = 3.2;      

% Tractor-Trailers Cornering Stiffness in [N/rad]
C1 = 150000;   
C2 = 121000;   
C3 = 121000;   
C4 = 121000;   
C5 = 121000;   

% Compute the state-space model, |G(s)|, of the lateral dynamics.
M = [m1 0 m2 0; 0 I1 -l6*m2 0; 0 0 -l7*m2 I2; 1 -l6 -1 -l6];
a = [(-C1-C2-C3)/Vx ((-C1*l1+C2*l2+C3*l3)/Vx)-m1*Vx (-C4-C5)/Vx...
    ((C4*l4+C5*l5)/Vx)-m2*Vx;(-C1*l1+C2*l2+C3*l3)/Vx ...
    (-C1*l1^2 - C2*l2^2 - C3*l3^2)/Vx ...
    (-C4*l6-C5*l6)./Vx l6*((-C4*l4-C5*l5)/Vx + m2*Vx);...
    0 0 (C4*l7+C5*l7+C4*l4+C5*l5)/Vx ...
    ((-C4*l4*l7-C5*l5*l7-C4*l4^2-C5*l5^2)/Vx+l7*m2*Vx);...
    0 -Vx 0 Vx];
b = [C1; C1*l1; 0; 0];
C = [1 0 0 0; 0 1 0 0];

A = M\a;
B = M\b;
end