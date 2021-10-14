#include "MotorEncoderFeedback.hpp"
constexpr size_t motorCount = 4U;
constexpr RotaryEncoder::MotorEncoderFeedbackConf confs[motorCount]{
    {
        //motorFL
        .encoder = { .pinA = 2, .pinB=4, .dirType=RotaryEncoder::DirType::RISING_CW },
        .motor = { .dirPin = 12, .pwmPin=8, .reverse = false }
    },
    {
        //motorFR
        .encoder = { .pinA = 3, .pinB=5, .dirType=RotaryEncoder::DirType::RISING_CW },
        .motor = { .dirPin = 13, .pwmPin=9, .reverse = false }
    },
    {
        //motorRL
        .encoder = { .pinA = 18, .pinB=20, .dirType=RotaryEncoder::DirType::RISING_CCW },
        .motor = { .dirPin = 14, .pwmPin=10, .reverse = true }
    },
    {
        //motorRR
        .encoder = { .pinA = 19, .pinB=21, .dirType=RotaryEncoder::DirType::RISING_CCW },
        .motor = { .dirPin = 15, .pwmPin=11, .reverse = true }
    }

};
using MyFeedbackTraits = RotaryEncoder::MotorEncoderFeedbackTraits<int32_t,float,1,8>;
using MyMotorEncoderFbManager = RotaryEncoder::MotorEncoderFbManager<confs,motorCount,MyFeedbackTraits>;
void setup()
{
    MyMotorEncoderFbManager::begin();
}

void loop()
{
    int32_t pos;
    int32_t posArray[4];
    int32_t tics_per_meter[4];
	MyMotorEncoderFbManager::fetchPosition<0>(pos);
    MyMotorEncoderFbManager::fetchPosition(posArray);
    MyMotorEncoderFbManager::setVelocity<2>(29.0f);
    float velocities[]={2.34f,2.23f,0.03f,4.20f};
    MyMotorEncoderFbManager::setVelocities(velocities);
    MyMotorEncoderFbManager::setTicksPerMeter(tics_per_meter);
    MyMotorEncoderFbManager::updateVelocities();
    
    delay(1000);
}
