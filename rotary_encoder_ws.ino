#include "RotaryEncoder.hpp"
constexpr RotaryEncoder::EncoderConf confs[] = { //interupt=pinA
                                        {.pinA=2  , .pinB=4  ,   .dirType=RotaryEncoder::DirType::RISING_CW  },
                                        {.pinA=3  , .pinB=5  ,   .dirType=RotaryEncoder::DirType::RISING_CW  },
                                        {.pinA=18 , .pinB=20 ,   .dirType=RotaryEncoder::DirType::RISING_CCW },
                                        {.pinA=19 , .pinB=21 ,   .dirType=RotaryEncoder::DirType::RISING_CCW },
                                     };

using MyEncoderManager = RotaryEncoder::EncoderManager<confs,4U,int32_t>;

void setup()
{
	MyEncoderManager::begin();
}

void loop()
{
    int32_t pos;
	// MyEncoderManager::fetchPosition<0>(pos);
    delay(1000);
}
