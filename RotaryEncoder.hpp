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

template <const EncoderConf* const conf,typename pos_t,size_t enc_id>
class RotaryEncoder_t{
    static constexpr EncoderConf enc_conf=conf[enc_id];
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    static pos_t pos;
    static_assert(enc_conf.dirType !=DirType::NONE,"Specify Direction type for all the encoders");
    static void init(){
        attachInterrupt(digitalPinToInterrupt(enc_conf.pinA),risingUpdate,RISING);
    }
    static void risingUpdate(){
        if(digitalRead(enc_conf.pinB)==(static_cast<uint8_t>(enc_conf.dirType)-1)){
                pos++;
            }
            else{
                pos--;
            }
    }
};

template <const EncoderConf* const conf,typename pos_t,size_t enc_id>
pos_t RotaryEncoder_t<conf,pos_t,enc_id>::pos = 0;

template<const EncoderConf* const conf,typename pos_t,size_t enc_id_rev,size_t count>
class EncoderInitializer{
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    static void init(){
        RotaryEncoder_t<conf,pos_t,count-enc_id_rev>::init();
        EncoderInitializer<conf,pos_t,enc_id_rev-1,count>::init();
    }
};
template<const EncoderConf* const conf,typename pos_t,size_t count>
class EncoderInitializer<conf,pos_t,1,count>{
    template<const EncoderConf* const  ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderInitializer;
    static void init(){
        RotaryEncoder_t<conf,pos_t,count-1>::init();
    }
};

template<const EncoderConf* const conf,typename pos_t,size_t enc_id_rev,size_t count>
class EncoderPosFetch{
    template<const EncoderConf* const ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    static void fetchPos(pos_t* posArray){
        posArray[count-enc_id_rev] = RotaryEncoder_t<conf,pos_t,count-enc_id_rev>::pos;
        EncoderPosFetch<conf,pos_t,enc_id_rev-1,count>::fetchPos(posArray);
    }
};
template<const EncoderConf* const conf,typename pos_t,size_t count>
class EncoderPosFetch<conf,pos_t,1,count>{
    template<const EncoderConf* const  ,size_t ,typename >
    friend struct EncoderManager;
    template<const EncoderConf* const  ,typename ,size_t ,size_t >
    friend class EncoderPosFetch;
    static void fetchPos(pos_t* posArray){
        posArray[count-1] = RotaryEncoder_t<conf,pos_t,count-1>::pos;
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
        pos=RotaryEncoder_t<encoderConfs,pos_t,enc_id>::pos;
    }
    static void fetchPosition(pos_t* posArray){
        EncoderPosFetch<encoderConfs,pos_t,count,count>::fetchPos(posArray);
    }
};

} // namespace RotaryEncoder
