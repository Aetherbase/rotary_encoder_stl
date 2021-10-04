#include "RotaryEncoder.hpp"
RotaryEncoder::EncoderConf confs[] = { //interupt=pinA
                                        {.pinA=2  , .pinB=4 },
                                        {.pinA=3  , .pinB=5 },
                                        {.pinA=18 , .pinB=20},
                                        {.pinA=19 , .pinB=21},
                                     };

using MyEncoderManager = RotaryEncoder::EncoderManager<confs,4U,int32_t>;

void setup()
{
    Serial.begin(9600);
	MyEncoderManager::begin();
}

void loop()
{
    int32_t pos;
	MyEncoderManager::fetchPosition<0>(pos);
    delay(1000);
}
