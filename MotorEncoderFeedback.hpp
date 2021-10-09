#include "RotaryEncoder.hpp"

namespace RotaryEncoder
{

template <const MotorEncoderFeedbackConf& conf, typename pos_t, typename vel_t>
class MotorEncoderFeedback{
    static constexpr MotorEncoderFeedbackConf enc_mot_conf=conf;
    using Encoder_t = RotaryEncoder_t<enc_mot_conf.encoder,pos_t>;
    static void init(){
        Encoder_t::init();
    };
    
    static vel_t velocity_target;
    static pos_t previous_pos;
    static pos_t ticks_per_meter;
    static unsigned long previous_time;
    static uint16_t previous_pwm;
    static vel_t pwm_by_tvel;
    static void setVelocity(vel_t velocity){
        velocity_target=velocity;
    };
    static void setTicksPerMeter(pos_t _ticks_per_meter){
        ticks_per_meter=_ticks_per_meter;
    }
    static void updateVelocity(){
        if(previous_time==0){
            previous_time=millis();
            return;
        }
        unsigned long current_time = millis();
        pos_t current_pos = Encoder_t::pos;
        unsigned long time_diff = current_time - previous_time;
        pos_t diff_pos = current_pos - previous_pos;
        vel_t prev_vel = ((vel_t)diff_pos)/((vel_t)time_diff);
        if((previous_pwm!=0)&&(prev_vel!=0)){
            pwm_by_tvel = ((vel_t)previous_pwm)/prev_vel;
        }
        uint16_t pwm_current = abs(pwm_by_tvel*((vel_t)ticks_per_meter)*velocity_target);
        analogWrite(conf.pwmPin,pwm_current);
        digitalWrite(conf.dirPin,conf.reverse!=(velocity_target>0));
        previous_pwm=pwm_current;
        previous_pos=current_pos;
        previous_time=current_time;
    };
};
}