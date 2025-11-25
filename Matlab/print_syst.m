function Q = print_syst()
    syms x y theta;  
    syms NTic_R NTic_L Enc_res;
    syms Rr Rl b;
    
    State = [x; y; theta];
    
    %Formulas from Antonelli and Chiaverini
    theta_incr = 2*pi * (NTic_R*Rr-NTic_L*Rl) / (Enc_res*b); 
    x_new = x + pi * ((NTic_R*Rr+NTic_L*Rl) / Enc_res) * cos(theta + theta_incr/2)
    y_new = y + pi * ((NTic_R*Rr+NTic_L*Rl) / Enc_res) * sin(theta + theta_incr/2)
    theta_new = theta + theta_incr
    
    enc_motion = [x ; y ; theta];
    
    Functions = [x_new; y_new; theta_new];
    no = 4096; % encoders resolution
    
    % Linearization
    J_symbolic = simplify(jacobian(Functions, State))

    % Uncertainties with encoder tics 
    % I assume the uncertainties as the encoders motin with one tic of the encoders
    K_calib = [0.8291  0.0873  0.0857]; % b Rr Rl
    theta_incr = subs(theta_incr, [NTic_R, NTic_L, b,  Rr, Rl, Enc_res], [1, 1, K_calib, no]);
    sigma_theta_ENC = double(subs(theta_new, [NTic_R, NTic_L, b,  Rr, Rl, Enc_res, theta], [1, -1, K_calib, no, 0]));
    
    sigma_x_ENC = double(subs(x_new, [NTic_R, NTic_L, b,  Rr, Rl, Enc_res, x, theta], [1, 1, K_calib, no, 0, 0]));
    sigma_y_ENC = double(subs(y_new, [NTic_R, NTic_L, b,  Rr, Rl, Enc_res, theta, y], [1, 1, K_calib, no, pi/2, 0]));
    variances_vector_ENC = [sigma_x_ENC, sigma_y_ENC, sigma_theta_ENC];
    Q = diag(variances_vector_ENC);

end