function param_calib = Calibration(K0, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)
options = optimoptions(@lsqnonlin, ...
    'Algorithm', 'levenberg-marquardt', ... % buon compromesso per problemi non lineari
    'Display', 'iter', ...                  % mostra l'andamento
    'StepTolerance', 1e-6, ...              % soglia ragionevole per variazione parametri
    'OptimalityTolerance', 1e-6, ...        % soglia per criterio di ottimalità
    'FunctionTolerance', 1e-6);             % soglia per variazione del valore funzione
    param_calib = lsqnonlin(@(K) Calib_lsq(K, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC), K0, [], [], options);

end

function mse_val = Calib_lsq(K, Ntic_L, Ntic_R, Enc_res, x_HTC, y_HTC, theta_HTC)

pose_Enc = EncodersMotion(K, Ntic_L, Ntic_R, Enc_res);

    x_ENC = pose_Enc(1,:)';
    y_ENC = pose_Enc(2,:)';
    theta_ENC = pose_Enc(3,:)';
    
    cost = [x_ENC; y_ENC; theta_ENC] - [x_HTC; y_HTC; theta_HTC];
    mse_val = mean(cost(:).^2);
        
end 