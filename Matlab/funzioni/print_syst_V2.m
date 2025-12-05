function [J_X, J_U] = print_syst_V2()
% PRINT_SYST_V2 Calcola le matrici Jacobiane simboliche per l'odometria
%
% OUTPUTS:
%   J_X       : Matrice Jacobiana rispetto allo Stato [x; y; theta] (3x3)
%   J_U       : Matrice Jacobiana rispetto al Controllo [NTic_R; NTic_L] (3x2)
%
% NOTE:
%   Utilizza il modello cinematico di Antonelli & Chiaverini basato su
%   integrazione Runge-Kutta di 2° ordine (metodo del punto medio).


    syms x y theta;  
    syms NTic_R NTic_L Enc_res;
    syms Rr Rl b;
    
    State = [x; y; theta];
    Control = [NTic_R; NTic_L];
    
    % Formule da Antonelli e Chiaverini (Runge-Kutta 2)
    theta_incr = 2*pi * (NTic_R*Rr-NTic_L*Rl) / (Enc_res*b); 
    
    % Posizione calcolata usando l'angolo medio (theta + theta_incr/2)
    x_new = x + pi * ((NTic_R*Rr+NTic_L*Rl) / Enc_res) * cos(theta + theta_incr/2);
    y_new = y + pi * ((NTic_R*Rr+NTic_L*Rl) / Enc_res) * sin(theta + theta_incr/2);
    theta_new = theta + theta_incr;
      
    Functions = [x_new; y_new; theta_new];
    
    % Linearizzazione Simbolica
    J_X = simplify(jacobian(Functions, State));
    J_U = simplify(jacobian(Functions, Control));
end