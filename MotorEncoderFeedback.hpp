#include "RotaryEncoder.hpp"
namespace RotaryEncoder
{
    template <typename _pos_t, typename _vel_t, uint32_t _def_pwm_by_tvel_num,uint32_t _def_pwm_by_tvel_denom>
    struct MotorEncoderFeedbackTraits
    {
        using vel_t = _vel_t;
        using pos_t = _pos_t;
        static constexpr vel_t def_pwm_by_tvel = ((vel_t)_def_pwm_by_tvel_num)/((vel_t)_def_pwm_by_tvel_denom);
    };
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    struct MotorEncoderFeedback_t
    {
        using meft = MotorEncoderFeedbackStatic_t;
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        using Encoder_t = RotaryEncoder_t<typename meft::Enc_t, pos_t>;
        static constexpr vel_t pwtvr_val = MEF_Traits::def_pwm_by_tvel;
        static vel_t velocity_target;
        static pos_t previous_pos;
        static pos_t ticks_per_meter;
        static unsigned long previous_time;
        static uint16_t previous_pwm;
        static vel_t pwm_by_tvel;

        static void init()
        {
            Encoder_t::init();
            pinMode(meft::dirPin, OUTPUT);
        };

        static void setVelocity(vel_t velocity)
        {
            velocity_target = velocity;
        };
        static void setTicksPerMeter(pos_t _ticks_per_meter)
        {
            ticks_per_meter = _ticks_per_meter;
        }
        static void updateVelocity()
        {
            if (previous_time == 0)
            {
                previous_time = 1000*millis();
                return;
            }
            unsigned long current_time = 1000*millis();
            pos_t current_pos = Encoder_t::pos;
            unsigned long time_diff = current_time - previous_time;
            pos_t diff_pos = current_pos - previous_pos;
            vel_t prev_vel = ((vel_t)diff_pos) / ((vel_t)time_diff);
            if ((previous_pwm != 0) && (prev_vel != 0))
            {
                pwm_by_tvel = ((vel_t)previous_pwm) / prev_vel;
            }
            uint16_t pwm_current = abs(pwm_by_tvel * ((vel_t)ticks_per_meter) * velocity_target);
            analogWrite(meft::pwmPin, pwm_current);
            digitalWrite(meft::dirPin, static_cast<uint8_t>(meft::reverse != (velocity_target > 0)));
            previous_pwm = pwm_current;
            previous_pos = current_pos;
            previous_time = current_time;
        };
    };
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    typename MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::vel_t\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::velocity_target = 0;
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    typename MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::pos_t\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::previous_pos = 0;
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    typename MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::pos_t\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::ticks_per_meter = 0;
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    unsigned long\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::previous_time = 0;
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    uint16_t\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::previous_pwm = 0;
    template <typename MotorEncoderFeedbackStatic_t, typename MEF_Traits>
    typename MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::vel_t\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::pwm_by_tvel =\
    MotorEncoderFeedback_t<MotorEncoderFeedbackStatic_t,MEF_Traits>::pwtvr_val;

    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t enc_id_rev, size_t count>
    struct MotorEncoderFbInitializer
    {
        constexpr static size_t enc_id = count - enc_id_rev;
        static void init()
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::init();
            MotorEncoderFbInitializer<conf, MEF_Traits, enc_id_rev - 1, count>::init();
        }
    };
    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t count>
    struct MotorEncoderFbInitializer<conf, MEF_Traits, 1, count>
    {
        constexpr static size_t enc_id = count - 1;
        static void init()
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::init();
        }
    };

    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t enc_id_rev, size_t count>
    struct MotorEncoderPosFetch
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - enc_id_rev;
        static void fetchPos(pos_t *posArray)
        {
            posArray[enc_id] = MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::Encoder_t::pos;
            MotorEncoderPosFetch<conf, MEF_Traits, enc_id_rev - 1, count>::fetchPos(posArray);
        }
    };
    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t count>
    struct MotorEncoderPosFetch<conf, MEF_Traits, 1, count>
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - 1;
        static void fetchPos(pos_t *posArray)
        {
            posArray[enc_id] = MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::Encoder_t::pos;
        }
    };

    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t enc_id_rev, size_t count>
    struct MotorEncoderVelocitySet
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - enc_id_rev;
        static void setVelocity(const vel_t *velArray)
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::setVelocity(velArray[enc_id]);
            MotorEncoderVelocitySet<conf, MEF_Traits, enc_id_rev - 1, count>::setVelocity(velArray);
        }
    };
    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t count>
    struct MotorEncoderVelocitySet<conf, MEF_Traits, 1, count>
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - 1;
        static void setVelocity(const vel_t *velArray)
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::setVelocity(velArray[enc_id]);
        }
    };

    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t enc_id_rev, size_t count>
    struct MotorEncoderTpmSet
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - enc_id_rev;
        static void setTicksPerMeter(const pos_t *ticks_per_meter)
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::setTicksPerMeter(ticks_per_meter[enc_id]);
            MotorEncoderTpmSet<conf, MEF_Traits, enc_id_rev - 1, count>::setTicksPerMeter(ticks_per_meter);
        }
    };
    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t count>
    struct MotorEncoderTpmSet<conf, MEF_Traits, 1, count>
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        constexpr static size_t enc_id = count - 1;
        static void setTicksPerMeter(const pos_t *ticks_per_meter)
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::setTicksPerMeter(ticks_per_meter[enc_id]);
        }
    };

    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t enc_id_rev, size_t count>
    struct MotorEncoderFbVelUpdater
    {
        constexpr static size_t enc_id = count - enc_id_rev;
        static void updateVel()
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::updateVelocity();
            MotorEncoderFbVelUpdater<conf, MEF_Traits, enc_id_rev - 1, count>::updateVel();
        }
    };
    template <const MotorEncoderFeedbackConf *const conf, typename MEF_Traits, size_t count>
    struct MotorEncoderFbVelUpdater<conf, MEF_Traits, 1, count>
    {
        constexpr static size_t enc_id = count - 1;
        static void updateVel()
        {
            MotorEncoderFeedback_t<MEFB_T(conf,enc_id),MEF_Traits>::updateVelocity();
        }
    };


    template <const MotorEncoderFeedbackConf *const confs, size_t count, typename MEF_Traits>
    struct MotorEncoderFbManager
    {
        using vel_t = typename MEF_Traits::vel_t;
        using pos_t = typename MEF_Traits::pos_t;
        static void begin(const pos_t *ticks_per_meter)
        {
            MotorEncoderFbInitializer<confs,MEF_Traits,count,count>::init();
            MotorEncoderTpmSet<confs,MEF_Traits,count,count>::setTicksPerMeter(ticks_per_meter);
        }
        template <size_t enc_id>
        static void fetchPosition(pos_t &pos)
        {
            static_assert(enc_id<count,"id should be less than count");
            pos = MotorEncoderFeedback_t<MEFB_T(confs,enc_id),MEF_Traits>::Encoder_t::pos;
        }
        static void fetchPosition(pos_t *posArray)
        {
            MotorEncoderPosFetch<confs,MEF_Traits,count,count>::fetchPos(posArray);
        }
        template <size_t enc_id>
        static void setVelocity(const vel_t &vel)
        {
            static_assert(enc_id<count,"id should be less than count");
            MotorEncoderFeedback_t<MEFB_T(confs,enc_id),MEF_Traits>::setVelocity(vel);
        }
        static void setVelocities(const vel_t *velArray)
        {
            MotorEncoderVelocitySet<confs,MEF_Traits,count,count>::setVelocity(velArray);
        }
        static void updateVelocities()
        {
            MotorEncoderFbVelUpdater<confs,MEF_Traits,count,count>::updateVel();
        }
    };
}