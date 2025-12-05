function plot_covariance_ellipses(x, y, Eps_x, n_sigma, step_plot)
% PLOT_COVARIANCE_ELLIPSES Disegna gli ellissi di incertezza sulla traiettoria
%
% INPUTS:
%   x, y      : Vettori della traiettoria (1xn)
%   Eps_x     : Matrice di covarianza 3D (3x3xn)
%   n_sigma   : Fattore di scala per la confidenza (es. 3 per 99.7%)
%   step_plot : Ogni quanti campioni disegnare un ellisse (es. 10 o 20)

    % Controllo preliminare: assicuriamoci di mantenere il grafico esistente
    hold on;

    % Numero totale di passi
    n = length(x);

    % Genera punti base del cerchio unitario (una volta sola)
    alpha_circ = linspace(0, 2*pi, 100);

    % Ciclo di plot
    for k = 1 : step_plot : n
        
        % 1. Estrai sottomatrice 2x2 (X, Y)
        % Eps_x è 3x3xn -> prendiamo indici 1:2, 1:2 all'istante k
        sxx = Eps_x(1, 1, k);
        syy = Eps_x(2, 2, k);
        sxy = Eps_x(1, 2, k); % sxy = syx
        
        % 2. Calcola l'angolo di orientamento (metodo analitico)
        angle = 0.5 * atan2(2 * sxy, sxx - syy);
        
        % 3. Calcola gli autovalori (Lunghezza assi^2)
        term1 = (sxx + syy) / 2;
        term2 = sqrt( ((sxx - syy) / 2)^2 + sxy^2 );
        
        lambda1 = term1 + term2; % Maggiore
        lambda2 = term1 - term2; % Minore
        
        % Protezione contro errori numerici (radici di numeri negativi)
        if lambda1 < 0, lambda1 = 0; end
        if lambda2 < 0, lambda2 = 0; end
        
        % 4. Calcola i semiassi scalati
        a = n_sigma * sqrt(lambda1); 
        b = n_sigma * sqrt(lambda2);
        
        % 5. Genera i punti locali (allineati a X)
        el_x_loc = a * cos(alpha_circ);
        el_y_loc = b * sin(alpha_circ);
        
        % 6. Ruota e Trasla (Coordinate Globali)
        % Applicazione della matrice di rotazione 2D esplicita
        mu_x = x(k);
        mu_y = y(k);
        
        plot_x = mu_x + el_x_loc * cos(angle) - el_y_loc * sin(angle);
        plot_y = mu_y + el_x_loc * sin(angle) + el_y_loc * cos(angle);
        
        % 7. Disegna
        plot(plot_x, plot_y, 'r-', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end
end

%[appendix]{"version":"1.0"}
%---
