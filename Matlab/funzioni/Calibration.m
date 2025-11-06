function param_calib = Calibration(K0, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)
% CALIBRATION - Estimate robot kinematic parameters by minimizing pose error
%
% This function estimates the calibration parameters of a differential-drive
% robot by minimizing the difference between the pose reconstructed from
% wheel encoder data and the ground-truth pose measured by an HTC Vive system.
%
% The optimization is performed using nonlinear least squares
% (Levenberg-Marquardt algorithm) via MATLAB's lsqnonlin function.
%
% INPUTS:
%   K0        - Initial guess of the calibration parameters [b, R_r, R_l]
%                b   = wheelbase [m]
%                R_r = right wheel radius [m]
%                R_l = left wheel radius [m]
%
%   Ntic_L    - Vector of left wheel encoder tick increments
%   Ntic_R    - Vector of right wheel encoder tick increments
%   Enc_res   - Encoder resolution [ticks/rev]
%
%   x_HTC     - Ground-truth x positions [m] from HTC Vive tracking
%   y_HTC     - Ground-truth y positions [m] from HTC Vive tracking
%   theta_HTC - Ground-truth orientation [rad] from HTC Vive tracking
%
% OUTPUT:
%   param_calib - Estimated calibration parameters [b, R_r, R_l]

options = optimoptions(@lsqnonlin, ...
    'Algorithm', 'levenberg-marquardt', ... % buon compromesso per problemi non lineari
    'Display', 'iter', ...                  % mostra l'andamento
    'StepTolerance', 1e-6, ...              % soglia ragionevole per variazione parametri
    'OptimalityTolerance', 1e-6, ...        % soglia per criterio di ottimalità
    'FunctionTolerance', 1e-6, ...          % soglia per variazione del valore funzione
    'MaxIterations', 5000, ...              % numero massimo di iterazioni
    'MaxFunctionEvaluations', 10000);        % numero massimo di valutazioni della funzio

    param_calib = lsqnonlin(@(K) Calib_lsq(K, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC), K0, [], [], options);

end


function mse_val = Calib_lsq(K, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)
% The cost function is defined as the mean squared error (MSE)

pose_Enc = EncodersMotion(K, Ntic_L, Ntic_R, Enc_res);

    x_ENC = pose_Enc(1,:)';
    y_ENC = pose_Enc(2,:)';
    theta_ENC = pose_Enc(3,:)';
    
    cost = [x_ENC; y_ENC; theta_ENC] - [x_HTC; y_HTC; theta_HTC];
    mse_val = mean(cost(:).^2);
        
end 