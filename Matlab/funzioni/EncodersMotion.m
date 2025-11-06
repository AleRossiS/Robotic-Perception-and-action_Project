function enc_motion = EncodersMotion(K, NTic_L, NTic_R, Enc_res)
% EncodersMotion - Compute robot pose from encoder ticks (Differential Drive)
%
% This function estimates the trajectory (x, y, theta) of a differential-drive
% robot using incremental encoder readings and the kinematic model
% described by Antonelli & Chiaverini.
%
% INPUTS:
%   K        = [b, R_r, R_l]   -> calibration parameters:
%                b   = wheelbase [m]
%                R_r = right wheel radius [m]
%                R_l = left wheel radius [m]
%   NTic_R   = vector of right wheel encoder tick increments
%   NTic_L   = vector of left wheel encoder tick increments
%   Enc_res  = encoder resolution [ticks/rev]
%
% OUTPUT:
%   enc_motion = [3 x N] matrix containing the estimated robot pose over time:
%                 row 1: x position [m]
%                 row 2: y position [m]
%                 row 3: orientation θ [rad]
%

    n = length(NTic_L);
    x = zeros(1, n);
    y = zeros(1, n);
    theta = zeros(1, n);
    theta_incr = zeros(1, n);


    %Formulas from Antonelli and Chiaverini 
    for i = 1:n-1
        
        theta_incr(i+1) = 2*pi * (NTic_R(i)*K(2)-NTic_L(i)*K(3)) / (Enc_res*K(1));
        
        x(i+1) = x(i) + pi * ((NTic_R(i)*K(2)+NTic_L(i)*K(3)) / Enc_res) * cos(theta(i) + theta_incr(i)/2);
        y(i+1) = y(i) + pi * ((NTic_R(i)*K(2)+NTic_L(i)*K(3)) / Enc_res) * sin(theta(i) + theta_incr(i)/2);
        theta(i+1) = theta(i) + theta_incr(i);
    end
    
    enc_motion = [x ; y ; theta];

end