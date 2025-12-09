/*
# FINAL EKF: SENSOR FUSION + AXIS ALIGNMENT + EXTRINSIC OFFSET
# Gestisce:
# 1. Inversione assi (se la RS guarda indietro).
# 2. Offset fisico (se la RS non è al centro del robot).
*/
#include <filter.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <deque>

#ifndef PLUGIN_NAME
#define PLUGIN_NAME "odometry_filter"
#endif

using namespace std;
using json = nlohmann::json;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class MovingAverage {
    std::deque<double> _window;
    size_t _size;
    double _sum;
public:
    MovingAverage(size_t size = 10) : _size(size), _sum(0.0) {}
    void resize(size_t new_size) { _size = new_size; _window.clear(); _sum = 0.0; }
    
    double update(double val) {
        _window.push_back(val);
        _sum += val;
        if (_window.size() > _size) {
            _sum -= _window.front();
            _window.pop_front();
        }
        return _sum / _window.size();
    }
};

class Odometry_filterPlugin : public Filter<json, json> {

    struct State {
        double x = 0.0, y = 0.0, theta = 0.0; 
        double Pxx = 0.01, Pyy = 0.01, Ptt = 0.01;
    } _state;

    struct Params {
        // --- CINEMATICA ---
        double wheel_radius_left = 0.0873; 
        double wheel_radius_right = 0.0857;
        double ticks_per_rev = 4096.0;
        double baseline = 0.8291;

        // --- TUNING EKF ---
        double sigma_v = 0.005;    
        double sigma_w = 0.002;    
        double sigma_rs_pos = 0.5; 
        double sigma_rs_ang = 0.1; 

        int filter_window = 10;

        // --- ALLINEAMENTO ASSI (Inversioni) ---
        bool invert_gyro = false;      
        bool invert_rs_x = false;      
        bool invert_rs_y = false;      
        bool invert_rs_theta = false;  
        
        // --- CALIBRAZIONE ESTRINSECA (Offset Sensore) ---
        // Distanza del sensore dal centro del robot (in metri)
        double cam_offset_x = 0.0; // Positivo se avanti, Negativo se indietro
        double cam_offset_y = 0.0; // Positivo se a sinistra
        double cam_yaw_offset = 0.0; // Rotazione della telecamera (es. 3.14)
    } _conf;

    long _incoming_ticks_l = 0, _incoming_ticks_r = 0;
    long _prev_ticks_l = 0, _prev_ticks_r = 0;
    bool _initialized = false;
    bool _has_new_encoder_data = false;

    double _current_gyro_z = 0.0;
    bool _has_gyro = false;

    MovingAverage _filter_x;
    MovingAverage _filter_y;
    MovingAverage _filter_theta;
    
    double _clean_rs_x = 0.0, _clean_rs_y = 0.0, _clean_rs_theta = 0.0;
    double _prev_raw_rs_x = -9999.0, _prev_raw_rs_y = -9999.0;
    bool _has_rs_update = false;

    double _last_timestamp = 0.0;
    double _prev_time = 0.0;
    string _last_agent_id = "";

public:
    string kind() override { return PLUGIN_NAME; }
    map<string, string> info() override { return {{"type", "EKF with Extrinsic Offset"}}; }

    Odometry_filterPlugin() : _filter_x(10), _filter_y(10), _filter_theta(10) {}

    return_type load_data(const json &in, std::string topic = "") override {
        try {
            double now = 0.0;
            if (in.contains("message") && in["message"].contains("timecode")) 
                now = in["message"]["timecode"].get<double>();
            else if (in.contains("/message/timecode")) 
                now = in["/message/timecode"].get<double>();
            if (now > 0) _last_timestamp = now;

            // 1. ENCODERS
            if (in.contains("/message/encoders/left") || (in.contains("message") && in["message"].contains("encoders"))) {
                if (in.contains("/message/encoders/left")) {
                    _incoming_ticks_l = (long)in["/message/encoders/left"].get<double>();
                    _incoming_ticks_r = (long)in["/message/encoders/right"].get<double>();
                } else {
                    _incoming_ticks_l = (long)in["message"]["encoders"]["left"].get<double>();
                    _incoming_ticks_r = (long)in["message"]["encoders"]["right"].get<double>();
                }
                
                if (!_initialized) {
                    _prev_ticks_l = _incoming_ticks_l;
                    _prev_ticks_r = _incoming_ticks_r;
                    _prev_time = _last_timestamp;
                    _initialized = true;
                } else {
                    _has_new_encoder_data = true;
                }
            }

            // 2. IMU (Gyro)
            if (in.contains("message") && in["message"].contains("gyro")) {
                auto& gyro = in["message"]["gyro"];
                if (gyro.is_array() && gyro.size() >= 3) {
                    double raw_gyro = gyro[2].get<double>();
                    _current_gyro_z = _conf.invert_gyro ? -raw_gyro : raw_gyro;
                    _has_gyro = true;
                }
            }

            // 3. REALSENSE
            double raw_rs_x = 0.0, raw_rs_y = 0.0, raw_rs_theta = 0.0;
            bool rs_found = false;

            if (in.contains("message") && in["message"].contains("pose")) {
                auto& p = in["message"]["pose"];
                if (p.contains("position")) {
                    auto& pos = p["position"];
                    if (pos.is_array() && pos.size() > 0) {
                        if (pos[0].is_array()) { 
                             raw_rs_x = pos[0][0].get<double>();
                             raw_rs_y = pos[0][1].get<double>();
                        } else { 
                             raw_rs_x = pos[0].get<double>();
                             raw_rs_y = pos[1].get<double>();
                        }
                        rs_found = true;
                    }
                }
                if (p.contains("attitude_along_z")) {
                    auto& att = p["attitude_along_z"];
                    if (att.is_array()) raw_rs_theta = att[0].get<double>();
                    else raw_rs_theta = att.get<double>();
                }
            }

            if (rs_found && (abs(raw_rs_x) > 0.001 || abs(raw_rs_y) > 0.001)) {
                // APPLICA INVERSIONI PRIMA DEL FILTRAGGIO
                if (_conf.invert_rs_x) raw_rs_x = -raw_rs_x;
                if (_conf.invert_rs_y) raw_rs_y = -raw_rs_y;
                if (_conf.invert_rs_theta) raw_rs_theta = -raw_rs_theta;
                
                raw_rs_theta += _conf.cam_yaw_offset;

                double dist = sqrt(pow(raw_rs_x - _prev_raw_rs_x, 2) + pow(raw_rs_y - _prev_raw_rs_y, 2));
                
                if (dist > 0.0001) {
                    _clean_rs_x = _filter_x.update(raw_rs_x);
                    _clean_rs_y = _filter_y.update(raw_rs_y);
                    _clean_rs_theta = _filter_theta.update(raw_rs_theta);
                    
                    _has_rs_update = true;
                    _prev_raw_rs_x = raw_rs_x;
                    _prev_raw_rs_y = raw_rs_y;
                }
            }

            if (in.contains("agent_id")) _last_agent_id = in["agent_id"].get<string>();

        } catch (...) { return return_type::error; }
        return return_type::success;
    }

    return_type process(json &out) override {
        out.clear();
        if (!_has_new_encoder_data) return return_type::success;

        long d_ticks_l = _incoming_ticks_l - _prev_ticks_l;
        long d_ticks_r = _incoming_ticks_r - _prev_ticks_r;
        _prev_ticks_l = _incoming_ticks_l;
        _prev_ticks_r = _incoming_ticks_r;
        _has_new_encoder_data = false;

        double dt = _last_timestamp - _prev_time;
        if (dt <= 0) dt = 0.04; 
        _prev_time = _last_timestamp;

        // --- PREDICTION (Stato al Centro del Robot) ---
        double d_left = (d_ticks_l / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_left;
        double d_right = (d_ticks_r / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_right;
        double ds = (d_right + d_left) / 2.0;

        double d_theta = 0.0;
        if (_has_gyro) {
            d_theta = _current_gyro_z * dt;
        } else {
            d_theta = (d_right - d_left) / _conf.baseline;
        }

        double avg_theta = _state.theta + d_theta / 2.0;
        _state.x += ds * cos(avg_theta);
        _state.y += ds * sin(avg_theta);
        _state.theta += d_theta;

        while (_state.theta > M_PI) _state.theta -= 2.0 * M_PI;
        while (_state.theta < -M_PI) _state.theta += 2.0 * M_PI;

        _state.Pxx += _conf.sigma_v * abs(ds);
        _state.Pyy += _conf.sigma_v * abs(ds);
        _state.Ptt += _conf.sigma_w * abs(d_theta);

        // --- CORRECTION (Con Compensazione Braccio di Leva) ---
        if (_has_rs_update) {
            
            // 1. Calcola dove DOVREBBE essere il sensore in base alla stima attuale
            // Pos_Sensore_Stimata = Pos_Robot_Stimata + Rotazione * Offset
            double est_sensor_x = _state.x + cos(_state.theta) * _conf.cam_offset_x - sin(_state.theta) * _conf.cam_offset_y;
            double est_sensor_y = _state.y + sin(_state.theta) * _conf.cam_offset_x + cos(_state.theta) * _conf.cam_offset_y;

            // 2. Calcola l'errore (Innovazione)
            // Differenza tra dove la RealSense dice di essere e dove noi stimiamo che sia
            double innov_x = _clean_rs_x - est_sensor_x;
            double innov_y = _clean_rs_y - est_sensor_y;

            // 3. Update Kalman
            double K_x = _state.Pxx / (_state.Pxx + _conf.sigma_rs_pos);
            double K_y = _state.Pyy / (_state.Pyy + _conf.sigma_rs_pos);

            // Correggiamo il CENTRO del robot basandoci sull'errore al SENSORE
            _state.x += K_x * innov_x;
            _state.y += K_y * innov_y;

            _state.Pxx *= (1.0 - K_x);
            _state.Pyy *= (1.0 - K_y);

            // Angolo
            double y_theta = _clean_rs_theta - _state.theta;
            while (y_theta > M_PI) y_theta -= 2.0 * M_PI;
            while (y_theta < -M_PI) y_theta += 2.0 * M_PI;

            double K_t = _state.Ptt / (_state.Ptt + _conf.sigma_rs_ang);
            _state.theta += K_t * y_theta;
            _state.Ptt *= (1.0 - K_t);

            _has_rs_update = false;
        }

        out["pose"]["position"]["x"] = _state.x;
        out["pose"]["position"]["y"] = _state.y;
        out["pose"]["position"]["z"] = 0.0;
        out["pose"]["orientation"]["yaw"] = _state.theta;
        
        std::vector<double> pos_vec = {_state.x, _state.y, 0.0};
        out["pose_vector"] = pos_vec;
        
        if (!_last_agent_id.empty()) out["source_id"] = _last_agent_id;
        out["sim_time"] = _last_timestamp;

        return return_type::success;
    }

    void set_params(void const *params) override {
        Filter::set_params(params);
        json p = *(json *)params;
        
        if (p.contains("wheel_radius_left")) _conf.wheel_radius_left = p["wheel_radius_left"];
        if (p.contains("wheel_radius_right")) _conf.wheel_radius_right = p["wheel_radius_right"];
        if (p.contains("baseline")) _conf.baseline = p["baseline"];
        
        if (p.contains("sigma_rs_pos")) _conf.sigma_rs_pos = p["sigma_rs_pos"];
        if (p.contains("sigma_rs_ang")) _conf.sigma_rs_ang = p["sigma_rs_ang"];
        if (p.contains("filter_window")) {
            _conf.filter_window = p["filter_window"];
            _filter_x.resize(_conf.filter_window);
            _filter_y.resize(_conf.filter_window);
            _filter_theta.resize(_conf.filter_window);
        }

        // Inversioni
        if (p.contains("invert_gyro")) _conf.invert_gyro = p["invert_gyro"];
        if (p.contains("invert_rs_x")) _conf.invert_rs_x = p["invert_rs_x"];
        if (p.contains("invert_rs_y")) _conf.invert_rs_y = p["invert_rs_y"];
        if (p.contains("invert_rs_theta")) _conf.invert_rs_theta = p["invert_rs_theta"];
        
        // Offset
        if (p.contains("cam_offset_x")) _conf.cam_offset_x = p["cam_offset_x"];
        if (p.contains("cam_offset_y")) _conf.cam_offset_y = p["cam_offset_y"];
        if (p.contains("cam_yaw_offset")) _conf.cam_yaw_offset = p["cam_yaw_offset"];
        
        _state = {0,0,0, 0.01, 0.01, 0.01};
        _initialized = false;
        _prev_raw_rs_x = -9999.0;
    }
};

INSTALL_FILTER_DRIVER(Odometry_filterPlugin, json, json)