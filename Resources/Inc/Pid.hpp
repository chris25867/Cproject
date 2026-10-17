#ifndef __PID_H__
#define __PID_H__

#include <cmath>

class Pid {
public:
    void Init(float kp, float ki, float kd, float dt,
              float max_output = 16384.0f, float min_output = -16384.0f);
    ~Pid() = default;

    void Control(float target, float current_data);
    float Result() const { return result_; }
    float Integral() const { return i_state_; }

    void SetFeedforward(float kff) { kff_ = kff; }                      
    void SetFriction(float fc, float dz);                       
    void SetStiction(float fs, float v_th);
    void SetDerivativeFilter(float n) { n_ = (n > 0.0f) ? n : 0.0f; }//微分滤波系数  
    void SetIntegralLimit(float lim);
    void Reset();                                                       
private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float dt_ = 0.001f;
    float last_error_ = 0.0f;
    float last_measure_ = 0.0f;
    float i_state_ = 0.0f;
    float d_filt_ = 0.0f;
    float result_ = 0.0f;
    float max_output_ = 16384.0f;
    float min_output_ = -16384.0f;

    float kff_ = 0.0f;
    float fc_ = 0.0f;
    float fs_ = 0.0f;
    float v_th_ = 0.0f;
    float fric_dead_zone_ = 0.0f;
    float n_ = 0.0f;
    float i_limit_ = 16384.0f;
};

inline void Pid::Init(float kp, float ki, float kd, float dt,
                      float max_output, float min_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = (dt > 0.0f) ? dt : 0.001f;//保证dt_大于0
    max_output_ = max_output;
    min_output_ = min_output;
    i_limit_ = (max_output > 0.0f) ? max_output : -max_output;

    Reset();//积分清零
}

inline void Pid::SetFriction(float fc, float dz) {
    fc_ = fc;
    fric_dead_zone_ = (dz > 0.0f) ? dz : 0.0f;//配置失误时归零关闭功能
}

inline void Pid::SetStiction(float fs, float v_th) {
    fs_ = (fs > 0.0f) ? fs : 0.0f;
    v_th_ = (v_th > 0.0f) ? v_th : 0.0f;
}

inline void Pid::SetIntegralLimit(float lim) {
    i_limit_ = (lim > 0.0f) ? lim : 0.0f;
}

inline void Pid::Reset() {
    last_error_ = 0.0f;
    last_measure_ = 0.0f;
    i_state_ = 0.0f;
    d_filt_ = 0.0f;
    result_ = 0.0f;
}

inline void Pid::Control(float target, float current_data) {
    const float error = target - current_data;

    const float d_raw = (current_data - last_measure_) / dt_;
    last_measure_ = current_data;

    if (n_ > 0.0f) {
        const float alpha = n_ / (n_ + 1.0f);
        d_filt_ += alpha * (d_raw - d_filt_);
    } else{
        d_filt_ = d_raw;
    }
    const float d_term = -kd_ * d_filt_;
    const float ff_term = kff_ * target;

    float r = 0.0f;
    if (fric_dead_zone_ > 0.0f) {
        r = target / fric_dead_zone_;
        if (r > 1.0f) {
            r = 1.0f;
        } else if (r < -1.0f) {
            r = -1.0f;
        }
    } else if (target > 0.0f) {
        r = 1.0f;
    } else if (target < 0.0f) {
        r = -1.0f;
    }

    float fric_term = fc_ * r;
    if (fs_ > 0.0f && v_th_ > 0.0f) {
        const float av = std::fabs(current_data);
        if (av < v_th_) {
            fric_term += fs_ * r * (1.0f - av / v_th_);
        }
    }

    const float p_term = kp_ * error;

    const float u_no_i = ff_term + p_term + d_term + fric_term;

    i_state_ += ki_ * error * dt_;
    if (i_state_ > i_limit_) {
        i_state_ = i_limit_;
    } else if (i_state_ < -i_limit_) {
        i_state_ = -i_limit_;
    }

    float output = u_no_i + i_state_;//u_no_i是分离积分的输出

    if (output > max_output_) {
        output = max_output_;
        i_state_ = max_output_ - u_no_i;//积分抗饱和
    } else if (output < min_output_) {
        output = min_output_;
        i_state_ = min_output_ - u_no_i;
    }

    if (i_state_ > i_limit_) {
        i_state_ = i_limit_;//u_no_i太大时积分小于零，相当于抵消一部分P或前馈太大造成的影响
    } else if (i_state_ < -i_limit_) {
        i_state_ = -i_limit_;
    }

    output = u_no_i + i_state_;
    if (output > max_output_) {
        output = max_output_;
    } else if (output < min_output_) {
        output = min_output_;
    }

    result_ = output;
    last_error_ = error;
}

#endif
