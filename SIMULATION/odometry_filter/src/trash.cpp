return_type process(json &out) override {
        out.clear();

        // 1. Check base: se non ci sono dati, esci.
        if (!_has_new_encoder_data) return return_type::success;

        // --- FIX CRUCIALE: FILTRO TEMPORALE (DEBOUNCING) ---
        double current_dt = _last_timestamp - _prev_time;
        
        // Se è passato meno di 0.01 secondi (10ms) dall'ultimo calcolo fisico,
        // NON fare nulla. Esci subito.
        // I tick continueranno ad accumularsi in _incoming_ticks grazie a load_data.
        // Al prossimo giro, il dt sarà > 0.01 e calcoleremo tutto insieme correttemente.
        if (current_dt < 0.01) {
            return return_type::success; 
        }
        // ---------------------------------------------------

        // Se siamo qui, il dt è valido (> 10ms). Procediamo!
        
        // Calcoliamo i delta accumulati dall'ultima volta
        long d_ticks_l = _incoming_ticks_l - _prev_ticks_l;
        long d_ticks_r = _incoming_ticks_r - _prev_ticks_r;
        
        // Aggiorniamo lo storico (Reset del conteggio differenziale)
        _prev_ticks_l = _incoming_ticks_l;
        _prev_ticks_r = _incoming_ticks_r;
        _has_new_encoder_data = false;

        // Aggiorniamo il tempo
        double dt = current_dt; 
        if (dt <= 1e-6) dt = 1e-6; // Paranoia check
        _prev_time = _last_timestamp;


        // --- DA QUI IN POI È TUTTO UGUALE A PRIMA ---
        // (Cinematica, Filtri, EKF...)
        
        double d_left = (d_ticks_l / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_left;
        double d_right = (d_ticks_r / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_right;
        double ds = (d_right + d_left) / 2.0;
        double ds_only_enc = ds;

        // --- CINEMATICA ROBUSTA (Low Pass + Clamp) ---
        double v_enc_raw = ds / dt;

        // 1. Filtro Passa-Basso (Alpha Filter)
        double alpha_v = 0.05; 
        _v_enc_smooth = (alpha_v * v_enc_raw) + ((1.0 - alpha_v) * _v_enc_smooth);

        // 2. Calcolo Accelerazione dalla velocità "Smooth"
        double a_enc_raw = (_v_enc_smooth - _prev_v_enc) / dt;
        
        // 3. Secondo stadio di filtraggio sull'accelerazione
        double alpha_a = 0.1; 
        _a_enc_smooth = (alpha_a * a_enc_raw) + ((1.0 - alpha_a) * _a_enc_smooth);
        
        // 4. PHYSICAL CLAMPING
        if (_a_enc_smooth > 20.0) _a_enc_smooth = 20.0;
        if (_a_enc_smooth < -20.0) _a_enc_smooth = -20.0;

        _prev_v_enc = _v_enc_smooth;

        // ... (Il resto della funzione process rimane identico al codice precedente) ...
        
        // Riporto solo la parte di logica anti-slip per completezza
        bool is_slipping = false;
        double min_imu_accel_for_correction = 0.2; 

        if(_conf.enable_slip_check && _bias_computed){ 
            if (abs(_current_accel_x) < _conf.static_thresh && abs(_current_gyro_z) < 0.01){
                if(abs(_v_enc_smooth) > 0.02){ 
                    is_slipping = true;
                    ds = 0.0;
                }
            }
            else if (abs(_a_enc_smooth) > abs(_current_accel_x) * _conf.slip_accel_ratio && 
                     abs(_current_accel_x) > min_imu_accel_for_correction) {
                is_slipping = true;
                double ratio = abs(_a_enc_smooth) / abs(_current_accel_x);
                if(ratio > 10.0) ratio = 10.0; 
                if(ratio < 1.0) ratio = 1.0; 
                ds = ds / ratio;
            }
        }
        
        // ... Log Debug ...
        _debug_slip.enc_accel = _a_enc_smooth;
        _debug_slip.imu_accel = _current_accel_x;
        _debug_slip.is_slipping = is_slipping;
        
        // ... (Continua con EKF Update, Correction, Output JSON) ...
        
        // --- COPIA IL RESTO DALLA TUA VERSIONE PRECEDENTE ---
        // (State update, RealSense correction, JSON output)
        
        // --- ESEMPIO PARTE FINALE NECESSARIA ---
        double d_theta_enc = (d_right - d_left) / _conf.baseline;
        double d_theta_gyro = 0.0;
        if(_has_gyro) d_theta_gyro = _current_gyro_z * dt;
        
        double var_enc = _conf.sigma_enc_rot * _conf.sigma_enc_rot;
        double var_gyro = _conf.sigma_gyro * _conf.sigma_gyro;
        double weight_gyro = 0.0; double weight_enc = 1.0;
        if(_has_gyro){ weight_gyro = var_enc/(var_enc+var_gyro); weight_enc = var_gyro/(var_enc+var_gyro); }
        double d_theta_fused = (d_theta_gyro * weight_gyro) + (d_theta_enc * weight_enc);
        
        double avg_theta_enc = _state_enc_only.theta + d_theta_enc / 2.0;
        _state_enc_only.x += ds_only_enc * cos(avg_theta_enc);
        _state_enc_only.y += ds_only_enc * sin(avg_theta_enc);
        _state_enc_only.theta += d_theta_enc;

        double avg_theta = _state.theta + d_theta_fused / 2.0;
        _state.x += ds * cos(avg_theta);
        _state.y += ds * sin(avg_theta);
        _state.theta += d_theta_fused;

        while (_state.theta > M_PI) _state.theta -= 2.0 * M_PI;
        while (_state.theta < -M_PI) _state.theta += 2.0 * M_PI;

        _state.Pxx += _conf.sigma_enc_lin * abs(ds);
        _state.Pyy += _conf.sigma_enc_lin * abs(ds);
        double sigma_curr = is_slipping ? 1.0 : _conf.sigma_v;
        _state.Pxx += sigma_curr * abs(ds);
        _state.Pyy += sigma_curr * abs(ds);
        _state.Ptt += _conf.sigma_w * abs(d_theta_fused);

        if(_has_rs_update){
             double est_sensor_x = _state.x + cos(_state.theta)*_conf.cam_offset_x - sin(_state.theta)*_conf.cam_offset_y;
             double est_sensor_y = _state.y + sin(_state.theta)*_conf.cam_offset_x + cos(_state.theta)*_conf.cam_offset_y;
             double innov_x = _rs_x - est_sensor_x;
             double innov_y = _rs_y - est_sensor_y;
             double K_x = _state.Pxx / (_state.Pxx + _conf.sigma_rs_pos);
             double K_y = _state.Pyy / (_state.Pyy + _conf.sigma_rs_pos);
             _state.x += K_x * innov_x; _state.y += K_y * innov_y;
             _state.Pxx *= (1.0 - K_x); _state.Pyy *= (1.0 - K_y);
             
             double y_theta = _rs_theta - _state.theta;
             while (y_theta > M_PI) y_theta -= 2.0*M_PI; while (y_theta < -M_PI) y_theta += 2.0*M_PI;
             double K_t = _state.Ptt / (_state.Ptt + _conf.sigma_rs_ang);
             _state.theta += K_t * y_theta;
             _state.Ptt *= (1.0 - K_t);
             _has_rs_update = false;
        }

        out["pose"]["position"]["x"] = _state.x;
        out["pose"]["position"]["y"] = _state.y;
        out["pose"]["position"]["z"] = 0.0;
        out["pose"]["orientation"]["yaw"] = _state.theta;
        out["pose_vector"] = std::vector<double>{_state.x, _state.y, 0.0};
        out["debug"]["raw_encoder_only"] = std::vector<double>{_state_enc_only.x, _state_enc_only.y, 0.0};
        out["debug"]["theta_enc"] = d_theta_enc;
        out["debug"]["theta_imu"] = d_theta_gyro;
        out["debug"]["accel_enc"] = _debug_slip.enc_accel;
        out["debug"]["accel_imu"] = _debug_slip.imu_accel;
        out["debug"]["is_slipping"] = _debug_slip.is_slipping ? 1.0 : 0.0;
        if (!_last_agent_id.empty()) out["source_id"] = _last_agent_id;
        out["sim_time"] = _last_timestamp;

        return return_type::success;
    }