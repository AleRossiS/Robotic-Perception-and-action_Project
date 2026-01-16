// RealSense
    if (in["message"]["pose"].contains("position") && in["message"]["pose"].contains("attitude")) {
      double raw_rs_x = 0.0, raw_rs_y = 0.0, raw_rs_theta = 0.0;
      auto& p = in["message"]["pose"];
      
      // 1. ESTRAZIONE DATI RAW
      if (p["position"][0].is_array()) {
          raw_rs_x = p["position"][0][0].get<double>();
          raw_rs_y = p["position"][0][1].get<double>();
      } else {
          raw_rs_x = p["position"][0].get<double>();
          raw_rs_y = p["position"][1].get<double>();
      }

      if(p["attitude"][0].is_array()){
        raw_rs_theta = p["attitude"][0][2].get<double>();
      } else {
        raw_rs_theta = p["attitude"][2].get<double>();
      }

      // 2. INNOVATION GATING (Controllo Qualità)
      bool accept_measurement = false;

      if (_first_rs_frame) {
          accept_measurement = true;
      } 
      else {
          // Calcola l'errore rispetto alla stima EKF (Giroscopio)
          double current_robot_theta = _state.x(2); 
          double candidate_theta = raw_rs_theta + _conf.rs_global_rotation; 
          candidate_theta = normalize_angle(candidate_theta); 
          
          double innovation_angle = candidate_theta - current_robot_theta;
          while (innovation_angle > M_PI) innovation_angle -= 2.0 * M_PI;
          while (innovation_angle < -M_PI) innovation_angle += 2.0 * M_PI;

          // Se l'errore è > 45 gradi, è un flip o un glitch -> RIFIUTA
          double angle_gate = 0.8; 
          if (abs(innovation_angle) > angle_gate) {
               accept_measurement = false;
          } else {
               accept_measurement = true;
          }
      }

      // 3. APPLICAZIONE MISURA (Solo se accettata)
      if (accept_measurement) {
          
          _last_input_rs_theta = raw_rs_theta; 
          
          // --- QUI MANCAVA LA MATEMATICA: L'HO REINSERITA ---
          double total_angle = _conf.rs_global_rotation; 
          double cos_rs = cos(total_angle);
          double sin_rs = sin(total_angle);

          // Rotazione base del dato
          _rs_x = (raw_rs_x * cos_rs - raw_rs_y * sin_rs);
          _rs_y = (raw_rs_x * sin_rs + raw_rs_y * cos_rs);

          double theta_new = raw_rs_theta + total_angle;
          while(theta_new > M_PI) theta_new -= 2.0 * M_PI;
          while(theta_new < -M_PI) theta_new += 2.0 * M_PI;

          _prev_rs_theta_raw = theta_new; // Per debug
          
          // Offset 180 gradi per l'angolo (specifico del tuo setup)
          _ekf_theta_rs = theta_new - M_PI; 
          while(_ekf_theta_rs > M_PI) _ekf_theta_rs -= 2.0 * M_PI;
          while(_ekf_theta_rs < -M_PI) _ekf_theta_rs += 2.0 * M_PI;
        
          _rs_theta = theta_new;
          // --------------------------------------------------

          _has_rs_update = true;
          _first_rs_frame = false;
          _aruco_valid_for_vis = true;
      
      } 
    }