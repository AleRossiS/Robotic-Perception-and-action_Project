% Error
function [max_error_pos, rms_error_pos] = CalculateError(x_calib, y_calib, theta_calib, x_ref, y_ref, theta_ref)
% Calcola l'errore di posizione e orientamento tra due traiettorie.
% Assumiamo che le traiettorie siano sincronizzate e abbiano la stessa lunghezza.

for i=1:length(x_calib)
    % Errore di Posizione (Euclideo)
    error_x(i) = x_calib(i) - x_ref(i);
    error_y(i) = y_calib(i) - y_ref(i);
    error_pos(i) = sqrt(error_x(i).^2 + error_y(i).^2);

    % Errore di Orientamento (Angolare)
    % Usiamo atan2 per gestire la periodicità di theta e calcoliamo la differenza angolare minima
    error_theta(i) = atan2(sin(theta_calib(i) - theta_ref(i)), cos(theta_calib(i) - theta_ref(i)));

    % Metriche di Errore di Posizione
    max_error_pos = max(error_pos);
    rms_error_pos = sqrt(mean(error_pos.^2)); % Errore Quadratico Medio (Root Mean Square)
end