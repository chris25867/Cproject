#ifndef GM6020_HPP
#define GM6020_HPP

#include <cstdint>

class GM6020 {
public:
    void Init(uint32_t id);
    ~GM6020() = default;
    uint32_t txId() const;
    uint32_t rxId() const;
    float angle() const;  
    float vel() const;     
    float current() const; 
    float temp() const;     
    void setInput(float current);
    bool encode(uint8_t *data);
    bool decode(uint8_t *data);
    
private:
    uint32_t  id_;
    float input_;
    float angle_;      
    float  vel_;
    float current_;
    float temp_;
    uint16_t last_ecd; 
    int32_t ecd_sum_;
    bool ecd_inited;
};

inline void GM6020::Init(uint32_t id)
{
    id_ = id;
    input_ = 0.0f;
    angle_ = 0.0f;
    vel_ = 0.0f;
    current_ = 0.0f;
    temp_ = 0.0f;
    last_ecd = 0;
    ecd_sum_ = 0;
    ecd_inited = false;
}

inline uint32_t GM6020::txId() const
{
    return 0x1FE;
}

inline uint32_t GM6020::rxId() const
{
    return 0x204U + id_;
}

inline float GM6020::angle() const
{
    return angle_;
}

inline float GM6020::vel() const
{
    return vel_;
}

inline float GM6020::current() const
{
    return current_;
}

inline float GM6020::temp() const
{
    return temp_;
}

inline void GM6020::setInput(float current)
{
    if (current > 16384.0f) {
        current = 16384.0f;
    } else if (current < -16384.0f) {
        current = -16384.0f;
    }

    input_ = current;
}

inline bool GM6020::encode(uint8_t *data)//打包为八字节数据
{
    if (!data) {
        return false;
    }

    int16_t current_cmd = static_cast<int16_t>(input_);

    switch (id_) {
    case 1:
        data[0] = (current_cmd >> 8) & 0xFF;
        data[1] = current_cmd & 0xFF;
        break;

    case 2:
        data[2] = (current_cmd >> 8) & 0xFF;
        data[3] = current_cmd & 0xFF;
        break;

    case 3:
        data[4] = (current_cmd >> 8) & 0xFF;
        data[5] = current_cmd & 0xFF;
        break;

    case 4:
        data[6] = (current_cmd >> 8) & 0xFF;
        data[7] = current_cmd & 0xFF;
        break;

    default:
        return false;
    }

    return true;
}

inline bool GM6020::decode(uint8_t *data)
{
    if (!data) {
        return false;
    }

    uint16_t ecd = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | data[1]);

    if (ecd > 8191U) {
        return false;
    }

    int16_t speed = static_cast<int16_t>(
        (static_cast<uint16_t>(data[2]) << 8) | data[3]);

    int16_t current_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) | data[5]);

    if (!ecd_inited) {
        last_ecd = ecd;
        ecd_sum_ = 0;
        ecd_inited = true;
    } else {
        int32_t delta = static_cast<int32_t>(ecd)
                      - static_cast<int32_t>(last_ecd);
        
        if (delta > 4096) {
            delta -= 8192;
        } else if (delta < -4096) {
            delta += 8192;
        }

        ecd_sum_ += delta;
        last_ecd = ecd;
    }

    angle_ = static_cast<float>(ecd_sum_)
           * (6.28318530718f / 8192.0f);

    vel_ = static_cast<float>(speed)
         * (6.28318530718f / 60.0f);

    current_ = static_cast<float>(current_raw);
    temp_ = static_cast<float>(data[6]);

    return true;
}

#endif