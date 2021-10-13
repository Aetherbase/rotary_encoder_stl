#pragma once
//Maintainer INFO: Harsh Davda <mailto:harsh.davda@gmail.com>
namespace RotaryEncoder
{
    

enum class DirType{
    RISING_CW=1,
    RISING_CCW=2,

    NONE=0
};


struct EncoderConf{
    const uint8_t pinA,pinB;
    const DirType dirType;
};

struct MotorEncoderFeedbackConf{
    EncoderConf encoder;
    uint8_t dirPin,pwmPin;
    bool reverse;
};

template <uint8_t pinA,uint8_t pinB, DirType dirType,typename pos_t>
class RotaryEncoder_t{
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    template <const MotorEncoderFeedbackConf& , typename , typename >
    friend class MotorEncoderFeedback;
    static pos_t pos;
    static_assert(dirType !=DirType::NONE,"Specify Direction type for all the encoders");
    static void init(){
        attachInterrupt(digitalPinToInterrupt(pinA),risingUpdate,RISING);
    }
    static void risingUpdate(){
        if(digitalRead(pinB)==(static_cast<uint8_t>(dirType)-1)){
                pos++;
            }
            else{
                pos--;
            }
    }
};

template <uint8_t pinA,uint8_t pinB, DirType dirType,typename pos_t>
pos_t RotaryEncoder_t<pinA,pinB,dirType,pos_t>::pos = 0;

template<const EncoderConf* const conf,typename pos_t,size_t enc_id_rev,size_t count>
class EncoderInitializer{
    constexpr static size_t enc_id = count-enc_id_rev;
    constexpr static EncoderConf enc_conf = conf[enc_id];
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    static void init(){
        RotaryEncoder_t<enc_conf.pinA,enc_conf.pinB,enc_conf.dirType,pos_t>::init();
        EncoderInitializer<conf,pos_t,enc_id_rev-1,count>::init();
    }
};
template<const EncoderConf* const conf,typename pos_t,size_t count>
class EncoderInitializer<conf,pos_t,1,count>{
    constexpr static size_t enc_id = count-1;
    constexpr static EncoderConf enc_conf = conf[enc_id];
    template<const EncoderConf* const  ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    static void init(){
        RotaryEncoder_t<enc_conf.pinA,enc_conf.pinB,enc_conf.dirType,pos_t>::init();
    }
};

template<const EncoderConf* const conf,typename pos_t,size_t enc_id_rev,size_t count>
class EncoderPosFetch{
    constexpr static size_t enc_id = count-enc_id_rev;
    constexpr static EncoderConf enc_conf = conf[enc_id];
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    static void fetchPos(pos_t* posArray){
        posArray[enc_id] = RotaryEncoder_t<enc_conf.pinA,enc_conf.pinB,enc_conf.dirType,pos_t>::pos;
        EncoderPosFetch<conf,pos_t,enc_id_rev-1,count>::fetchPos(posArray);
    }
};
template<const EncoderConf* const conf,typename pos_t,size_t count>
class EncoderPosFetch<conf,pos_t,1,count>{
    constexpr static size_t enc_id = count-1;
    constexpr static EncoderConf enc_conf = conf[enc_id];
    template<const EncoderConf* const  ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    static void fetchPos(pos_t* posArray){
        posArray[enc_id] = RotaryEncoder_t<enc_conf.pinA,enc_conf.pinB,enc_conf.dirType,pos_t>::pos;
    }
};

template<const EncoderConf* const encoderConfs,size_t count,typename pos_t>
struct EncoderManager{
    static void begin(){
        EncoderInitializer<encoderConfs,pos_t,count,count>::init();
    }
    template <size_t enc_id>
    static void fetchPosition(pos_t& pos){
        static_assert(enc_id<count,"id should be less than count");
        constexpr EncoderConf conf = encoderConfs[enc_id];
        pos=RotaryEncoder_t<conf.pinA,conf.pinB,conf.dirType,pos_t>::pos;
    }
    static void fetchPosition(pos_t* posArray){
        EncoderPosFetch<encoderConfs,pos_t,count,count>::fetchPos(posArray);
    }
};

} // namespace RotaryEncoder
