#pragma once
//Maintainer INFO: Harsh Davda <mailto:harsh.davda@gmail.com>
namespace RotaryEncoder
{

    using PinType = uint8_t;
    enum class DirType
    {
        RISING_CW = 1,
        RISING_CCW = 2,

        NONE = 0
    };

    struct EncoderConf
    {
        const PinType pinA, pinB;
        const DirType dirType;
    };
    struct MotorConf
    {
        const PinType dirPin, pwmPin;
        const bool reverse;
    };
    struct MotorEncoderFeedbackConf
    {
        const EncoderConf encoder;
        const MotorConf motor;
    };

    template <PinType _pinA, PinType _pinB, DirType _dirType>
    struct EncoderConfStatic
    {
        static constexpr PinType pinA = _pinA;
        static constexpr PinType pinB = _pinB;
        static constexpr DirType dirType = _dirType;
    };
    template <typename EncoderConfStatic_t, PinType _dirPin, PinType _pwmPin, bool _reverse>
    struct MotorEncoderFeedbackStatic
    {
        using Enc_t = EncoderConfStatic_t;
        static constexpr PinType pinA = EncoderConfStatic_t::pinA;
        static constexpr PinType pinB = EncoderConfStatic_t::pinB;
        static constexpr DirType dirType = EncoderConfStatic_t::dirType;
        static constexpr PinType dirPin = _dirPin;
        static constexpr PinType pwmPin = _pwmPin;
        static constexpr bool reverse = _reverse;
    };
#define ENC_T_C(conf) EncoderConfStatic<conf.pinA, conf.pinB, conf.dirType>
#define ENC_T(conf, id) ENC_T_C(conf[id])
#define MEFB_T_C(conf) MotorEncoderFeedbackStatic<ENC_T_C(conf.encoder), conf.motor.dirPin, conf.motor.pwmPin, conf.motor.reverse>
#define MEFB_T(conf, id) MEFB_T_C(conf[id])

    template <typename EncoderConfStatic_t, typename pos_t>
    struct RotaryEncoder_t
    {
        using ect = EncoderConfStatic_t;
        static pos_t pos;
        static_assert(ect::dirType != DirType::NONE, "Specify Direction type for all the encoders");
        static void init()
        {
            attachInterrupt(digitalPinToInterrupt(ect::pinA), risingUpdate, RISING);
        }
        static void risingUpdate()
        {
            if (digitalRead(ect::pinB) == (static_cast<uint8_t>(ect::dirType) - 1))
            {
                pos++;
            }
            else
            {
                pos--;
            }
        }
    };

    template <typename EncoderConfStatic_t, typename pos_t>
    pos_t RotaryEncoder_t<EncoderConfStatic_t, pos_t>::pos = 0;

    template <const EncoderConf *const conf, typename pos_t, size_t enc_id_rev, size_t count>
    struct EncoderInitializer
    {
        constexpr static size_t enc_id = count - enc_id_rev;
        static void init()
        {
            RotaryEncoder_t<ENC_T(conf, enc_id), pos_t>::init();
            EncoderInitializer<conf, pos_t, enc_id_rev - 1, count>::init();
        }
    };
    template <const EncoderConf *const conf, typename pos_t, size_t count>
    struct EncoderInitializer<conf, pos_t, 1, count>
    {
        constexpr static size_t enc_id = count - 1;
        static void init()
        {
            RotaryEncoder_t<ENC_T(conf, enc_id), pos_t>::init();
        }
    };

    template <const EncoderConf *const conf, typename pos_t, size_t enc_id_rev, size_t count>
    struct EncoderPosFetch
    {
        constexpr static size_t enc_id = count - enc_id_rev;
        static void fetchPos(pos_t *posArray)
        {
            posArray[enc_id] = RotaryEncoder_t<ENC_T(conf, enc_id), pos_t>::pos;
            EncoderPosFetch<conf, pos_t, enc_id_rev - 1, count>::fetchPos(posArray);
        }
    };
    template <const EncoderConf *const conf, typename pos_t, size_t count>
    struct EncoderPosFetch<conf, pos_t, 1, count>
    {
        constexpr static size_t enc_id = count - 1;
        static void fetchPos(pos_t *posArray)
        {
            posArray[enc_id] = RotaryEncoder_t<ENC_T(conf, enc_id), pos_t>::pos;
        }
    };

    template <const EncoderConf *const encoderConfs, size_t count, typename pos_t>
    struct EncoderManager
    {
        static void begin()
        {
            EncoderInitializer<encoderConfs, pos_t, count, count>::init();
        }
        template <size_t enc_id>
        static void fetchPosition(pos_t &pos)
        {
            static_assert(enc_id < count, "id should be less than count");
            pos = RotaryEncoder_t<ENC_T(encoderConfs, enc_id), pos_t>::pos;
        }
        static void fetchPosition(pos_t *posArray)
        {
            EncoderPosFetch<encoderConfs, pos_t, count, count>::fetchPos(posArray);
        }
    };

} // namespace RotaryEncoder
