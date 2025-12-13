/*
# FINAL EKF: FUSION + AXIS ALIGN + IMU SLIP DETECTION
# 1. Rotazione e Posizione RS (Correzione lenta)
# 2. Gyroscope Z (Rotazione veloce)
# 3. Accelerometer X (Anti-Slip / Motion Validation) <--- NEW!
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
public:
    MovingAverage(size_t size = 10) : _size(size) {}
    void resize(size_t new_size) { _size = new_size; _window.clear(); }
    
    double update(double val) {
        _window.push_back(val);
        if (_window.size() > _size) _window.pop_front();
        double sum = 0;
        for(double v : _window) sum += v;
        return sum / _window.size();
    }
};

class Odometry_filterPlugin : public Filter<json, json> {

    struct State {
        double x = 0.0, y = 0.0, theta = 0.0; 
        double Pxx = 0.01, Pyy = 0.01, Ptt = 0.01;
    } _state;

    // Struttura per debuggare lo slip
    struct DebugSlip {
        double enc_accel = 0.0;
        double imu_accel = 0.0;
        bool is_slipping = false;
    } _debug_slip;

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

        // --- SLIP DETECTION (NEW) ---
        double slip_accel_thresh = 0.2; // Soglia accelerazione IMU per confermare moto (m/s^2)
        double static_thresh = 0.05;    // Se accel < di questo, siamo fermi
        bool enable_slip_check = true;

        // Filtri
        int filter_window_rs = 10;
        int filter_window_imu = 5; 
        int filter_window_acc = 10; // Finestra per accelerometro

        // Geometria
        double rs_global_rotation = 0.0; 
        double cam_offset_x = 0.0; 
        double cam_offset_y = 0.0;
        bool invert_gyro = true; 
    } _conf;

    long _incoming_ticks_l = 0, _incoming_ticks_r = 0;
    long _prev_ticks_l = 0, _prev_ticks_r = 0;
    bool _initialized = false;
    bool _has_new_encoder_data = false;

    // IMU Data (Gyro + Accel)
    MovingAverage _filter_gyro;
    MovingAverage _filter_accel; // Filtro per Accel X
    double _clean_gyro_z = 0.0;
    double _clean_accel_x = 0.0;
    bool _has_imu = false;

    // Encoder Dynamics
    double _prev_v_enc = 0.0; // Velocità precedente (per calcolo accel)

    // RealSense Data
    MovingAverage _filter_x, _filter_y, _filter_theta;
    double _clean_rs_x = 0.0, _clean_rs_y = 0.0, _clean_rs_theta = 0.0;
    double _prev_raw_rs_x = -9999.0;
    bool _has_rs_update = false;

    double _last_timestamp = 0.0;
    double _prev_time = 0.0;
    string _last_agent_id = "";

public:
    string kind() override { return PLUGIN_NAME; }
    map<string, string> info() override { return {{"type", "EKF with Anti-Slip Logic"}}; }

    Odometry_filterPlugin() : _filter_x(10), _filter_y(10), _filter_theta(10), _filter_gyro(5), _filter_accel(10) {}

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

            // 2. IMU (Gyro + Accel)
            if (in.contains("message")) {
                // Gyro
                if (in["message"].contains("gyro")) {
                    auto& gyro = in["message"]["gyro"];
                    if (gyro.is_array() && gyro.size() >= 3) {
                        double raw_gyro = gyro[2].get<double>();
                        if (_conf.invert_gyro) raw_gyro = -raw_gyro;
                        _clean_gyro_z = _filter_gyro.update(raw_gyro);
                        _has_imu = true;
                    }
                }
                // Accel (NEW!) - Leggiamo asse X (Indice 0)
                if (in["message"].contains("accel")) {
                    auto& acc = in["message"]["accel"];
                    if (acc.is_array() && acc.size() >= 1) {
                        double raw_acc_x = acc[0].get<double>();
                        _clean_accel_x = _filter_accel.update(raw_acc_x);
                    }
                }
            }

            // 3. REALSENSE
            double raw_rs_x = 0.0, raw_rs_y = 0.0, raw_rs_theta = 0.0;
            bool rs_found = false;

            if (in.contains("message") && in["message"].contains("pose")) {
                auto& p = in["message"]["pose"];
                if (p.contains("position") && p["position"].is_array() && p["position"].size() > 0) {
                    if (p["position"][0].is_array()) { 
                         raw_rs_x = p["position"][0][0].get<double>();
                         raw_rs_y = p["position"][0][1].get<double>();
                    } else { 
                         raw_rs_x = p["position"][0].get<double>();
                         raw_rs_y = p["position"][1].get<double>();
                    }
                    rs_found = true;
                }
                if (p.contains("attitude_along_z")) {
                    auto& att = p["attitude_along_z"];
                    if (att.is_array()) raw_rs_theta = att[0].get<double>();
                    else raw_rs_theta = att.get<double>();
                }
            }

            if (rs_found && (abs(raw_rs_x) > 0.001 || abs(raw_rs_y) > 0.001)) {
                if (abs(_conf.rs_global_rotation) > 0.001) {
                    double rot = _conf.rs_global_rotation;
                    double x_new = raw_rs_x * cos(rot) - raw_rs_y * sin(rot);
                    double y_new = raw_rs_x * sin(rot) + raw_rs_y * cos(rot);
                    raw_rs_x = x_new; raw_rs_y = y_new; raw_rs_theta += rot;
                }
                
                double dist = sqrt(pow(raw_rs_x - _prev_raw_rs_x, 2));
                if (dist > 0.0001) {
                    _clean_rs_x = _filter_x.update(raw_rs_x);
                    _clean_rs_y = _filter_y.update(raw_rs_y);
                    _clean_rs_theta = _filter_theta.update(raw_rs_theta);
                    _has_rs_update = true;
                    _prev_raw_rs_x = raw_rs_x;
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

        // --- CALCOLO CINEMATICA ENCODER ---
        double d_left = (d_ticks_l / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_left;
        double d_right = (d_ticks_r / _conf.ticks_per_rev) * 2.0 * M_PI * _conf.wheel_radius_right;
        
        // Spostamento teorico (se non slittasse)
        double ds = (d_right + d_left) / 2.0;
        
        // Calcolo Velocità e Accelerazione Encoder
        double v_enc = ds / dt;
        double a_enc = (v_enc - _prev_v_enc) / dt;
        _prev_v_enc = v_enc;

        // --- SLIP DETECTION (Logic Core) ---
        bool is_slipping = false;
        
        if (_conf.enable_slip_check) {
            // 1. ZUPT (Zero Velocity Update)
            // Se l'IMU dice che l'accelerazione è NULLA e il Gyro è fermo...
            // ...ma gli encoder segnano movimento -> Slittamento da fermo!
            if (abs(_clean_accel_x) < _conf.static_thresh && abs(_clean_gyro_z) < 0.01) {
                if (abs(v_enc) > 0.001) {
                    is_slipping = true; 
                    ds = 0.0; // FORZA FERMO
                }
            }
            // 2. Acceleration Mismatch (Burnout Check)
            // Se gli encoder dicono "Partenza a razzo" (alta accel)
            // Ma l'IMU dice "Movimento blando"
            else if (abs(a_enc) > 0.5 && abs(_clean_accel_x) < _conf.slip_accel_thresh) {
                is_slipping = true;
                ds *= 0.1; // Riduci drasticamente il movimento (frena la stima)
            }
        }

        // Export debug info
        _debug_slip.enc_accel = a_enc;
        _debug_slip.imu_accel = _clean_accel_x;
        _debug_slip.is_slipping = is_slipping;

        // --- PREDICTION (Usa Gyro) ---
        double d_theta = 0.0;
        if (_has_imu) {
            d_theta = _clean_gyro_z * dt; 
        } else {
            d_theta = (d_right - d_left) / _conf.baseline;
        }

        double avg_theta = _state.theta + d_theta / 2.0;
        _state.x += ds * cos(avg_theta);
        _state.y += ds * sin(avg_theta);
        _state.theta += d_theta;

        while (_state.theta > M_PI) _state.theta -= 2.0 * M_PI;
        while (_state.theta < -M_PI) _state.theta += 2.0 * M_PI;

        // Aumenta incertezza se stiamo slittando
        double sigma_curr = is_slipping ? 1.0 : _conf.sigma_v; 
        _state.Pxx += sigma_curr * abs(ds);
        _state.Pyy += sigma_curr * abs(ds);
        _state.Ptt += _conf.sigma_w * abs(d_theta);

        // --- CORRECTION ---
        if (_has_rs_update) {
            double est_sensor_x = _state.x + cos(_state.theta) * _conf.cam_offset_x - sin(_state.theta) * _conf.cam_offset_y;
            double est_sensor_y = _state.y + sin(_state.theta) * _conf.cam_offset_x + cos(_state.theta) * _conf.cam_offset_y;

            double innov_x = _clean_rs_x - est_sensor_x;
            double innov_y = _clean_rs_y - est_sensor_y;

            double K_x = _state.Pxx / (_state.Pxx + _conf.sigma_rs_pos);
            double K_y = _state.Pyy / (_state.Pyy + _conf.sigma_rs_pos);

            _state.x += K_x * innov_x;
            _state.y += K_y * innov_y;

            _state.Pxx *= (1.0 - K_x);
            _state.Pyy *= (1.0 - K_y);

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
        out["pose_vector"] = std::vector<double>{_state.x, _state.y, 0.0};
        
        // Debug Outputs
        out["debug"]["accel_enc"] = _debug_slip.enc_accel;
        out["debug"]["accel_imu"] = _debug_slip.imu_accel;
        out["debug"]["is_slipping"] = _debug_slip.is_slipping ? 1.0 : 0.0;
        
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
        if (p.contains("filter_window_rs")) _conf.filter_window_rs = p["filter_window_rs"];
        if (p.contains("filter_window_imu")) _conf.filter_window_imu = p["filter_window_imu"];
        
        if (p.contains("rs_global_rotation")) _conf.rs_global_rotation = p["rs_global_rotation"];
        if (p.contains("cam_offset_x")) _conf.cam_offset_x = p["cam_offset_x"];
        if (p.contains("invert_gyro")) _conf.invert_gyro = p["invert_gyro"];

        // Slip params
        if (p.contains("slip_accel_thresh")) _conf.slip_accel_thresh = p["slip_accel_thresh"];
        if (p.contains("static_thresh")) _conf.static_thresh = p["static_thresh"];
        if (p.contains("enable_slip_check")) _conf.enable_slip_check = p["enable_slip_check"];
        
        _state = {0,0,0, 0.01, 0.01, 0.01};
        _filter_x.resize(_conf.filter_window_rs);
        _filter_y.resize(_conf.filter_window_rs);
        _filter_theta.resize(_conf.filter_window_rs);
        _filter_gyro.resize(_conf.filter_window_imu);
        _filter_accel.resize(_conf.filter_window_acc);
        
        _initialized = false;
        _prev_raw_rs_x = -9999.0;
    }
};

INSTALL_FILTER_DRIVER(Odometry_filterPlugin, json, json)