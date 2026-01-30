// ... (inizio process invariato) ...
        
        // =========================================================
        // 1. CALCOLO SPOSTAMENTI RAW (Encoder)
        // =========================================================
        double d_left = (d_ticks_l / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_left;
        double d_right = (d_ticks_r / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_right;
        
        double ds_enc_raw = (d_right + d_left) / 2.0; 
        
        // Calcolo rotazione e velocità angolare encoder per il confronto
        double d_theta_enc_raw = (d_right - d_left) / _conf.baseline;
        double omega_enc_raw = d_theta_enc_raw / dt; 

        // =========================================================
        // 2. FILTRI PER SLIP CHECK
        // =========================================================
        // (Usa i filtri che abbiamo definito prima per encoder e imu)
        // ... (Logica filtri _v_enc_smooth, a_enc_final, a_imu_final) ...
        // [Copia la parte dei filtri dalla risposta precedente]

        // =========================================================
        // 3. SLIP CHECK ESTESO (FEEDBACK PROF)
        // =========================================================
        
        bool is_slipping = false;
        
        // A. Controllo Accelerazione (Lineare)
        // Le ruote dicono che accelero, l'IMU dice che sono fermo -> Slitto
        double accel_diff = abs(a_enc_final - a_imu_final);
        bool slip_linear = (accel_diff > 1.0); 

        // B. Controllo Velocità Angolare (Rotazionale) - NUOVO!
        // Le ruote dicono che giro (es. una avanti una indietro), l'IMU dice che sono fermo.
        // O le ruote dicono 0 e il robot gira (scivola lateralmente - meno probabile su differenziale)
        
        // _current_gyro_z è la velocità angolare reale (calibrata)
        double omega_diff = abs(omega_enc_raw - _current_gyro_z);
        
        // Soglia: 0.5 rad/s sono circa 30 gradi/s.
        // Se la differenza è maggiore, una delle due fonti mente.
        bool slip_angular = (omega_diff > 0.5); 

        // Logica combinata
        if (_conf.enable_slip_check && _bias_computed) {
            // Se stiamo curvando forte (es. > 1 rad/s), l'accelerazione lineare è inaffidabile 
            // a causa della forza centripeta, quindi ignoriamo il check LINEARE ma teniamo quello ANGOLARE.
            bool is_turning_fast = abs(_current_gyro_z) > 1.0; 
            
            if (!is_turning_fast && slip_linear) is_slipping = true;
            if (slip_angular) is_slipping = true;
        }

        // --- GESTIONE DELLO SLIP ---
        double ds_final;
        double d_theta_final;
        
        if (is_slipping) {
            // Se slitto, mi fido dell'inerzia (IMU)
            // Integro velocità precedente + accelerazione IMU
            double v_imu_pred = _fused_velocity + (a_imu_final * dt);
            ds_final = v_imu_pred * dt;
            _fused_velocity = v_imu_pred;

            // Se slip angolare, uso SOLO giroscopio
            d_theta_final = _current_gyro_z * dt; 
            
            out["debug"]["slip_reason"] = slip_angular ? "ANGULAR" : "LINEAR";
        } 
        else {
            // Normale: Encoder comandano
            ds_final = ds_enc_raw;
            _fused_velocity = ds_enc_raw / dt;
            
            // Fusione angolare classica (Encoder + Gyro pesati)
            double d_theta_gyro = _current_gyro_z * dt;
            
            // Calcolo pesi varianza
            double var_enc = _conf.sigma_enc_rot * _conf.sigma_enc_rot;
            double var_gyro = _conf.sigma_gyro * _conf.sigma_gyro;
            double w_gyro = var_enc / (var_enc + var_gyro);
            double w_enc = var_gyro / (var_enc + var_gyro);
            
            d_theta_final = (d_theta_gyro * w_gyro) + (d_theta_enc_raw * w_enc);
            
            out["debug"]["slip_reason"] = "NONE";
        }

        // =========================================================
        // 4. PREDIZIONE EKF: MODELLO DI ERRORE CUMULATIVO (FEEDBACK PROF)
        // =========================================================
        
        // Qui definiamo "segmento per segmento" quanto errore aggiungiamo.
        // L'errore NON deve essere fisso (es. 0.01), ma proporzionale a quanto ci muoviamo.
        
        // Errore Lineare: "Sbaglio del 5% sulla distanza percorsa"
        // _conf.sigma_v viene interpretato come percentuale (es. 0.05)
        double motion_noise_lin = is_slipping ? 0.5 : _conf.sigma_v; // Se slitto sono incerto al 50%
        double sigma_ds_segment = (motion_noise_lin * abs(ds_final)) + 0.001; 

        // Errore Angolare: "Sbaglio del 5% sull'angolo girato + errore fisso del gyro"
        // _conf.sigma_w come percentuale (es. 0.05)
        double sigma_dtheta_segment = (_conf.sigma_w * abs(d_theta_final)) + (0.002 * dt); 

        // Passiamo questi errori "del segmento" alla funzione predict.
        // La funzione predict farà: P_totale = F * P_totale * F' + Q(sigma_segment)
        // Così l'incertezza si accumula correttamente.
        
        ekf_predict(_state, ds_final, d_theta_final, sigma_ds_segment, sigma_dtheta_segment);
        ekf_predict(_state_partial, ds_final, d_theta_final, sigma_ds_segment, sigma_dtheta_segment);

        // ... (Prosegui con Update RealSense e Output) ...