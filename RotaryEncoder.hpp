#pragma once
//
namespace RotaryEncoder
{
    
struct EncoderConf{
    uint8_t pinA,pinB;
};

template <EncoderConf* conf,typename pos_t,size_t enc_id>
struct RotaryEncoder_t{
    static pos_t pos;
    static void init(){
        attachInterrupt(digitalPinToInterrupt(conf[enc_id].pinA),risingUpdate,RISING);
    }
    static void risingUpdate(){
        if(digitalRead(conf[enc_id].pinB)==0){
                pos++;
            }
            else{
                pos--;
            }
    }
};

template <EncoderConf* conf,typename pos_t,size_t enc_id>
pos_t RotaryEncoder_t<conf,pos_t,enc_id>::pos = 0;

template<EncoderConf* conf,typename pos_t,size_t enc_id_rev,size_t count>
struct EncoderInitializer{
    static void init(){
        Serial.print("Initializing encoder id: ");
        Serial.println(count-enc_id_rev);
        RotaryEncoder_t<conf,pos_t,count-enc_id_rev>::init();
        EncoderInitializer<conf,pos_t,enc_id_rev-1,count>::init();
    }
};
template<EncoderConf* conf,typename pos_t,size_t count>
struct EncoderInitializer<conf,pos_t,1,count>{
    static void init(){
      Serial.print("Initializing encoder id: ");
        Serial.println(count-1);
        RotaryEncoder_t<conf,pos_t,count-1>::init();
    }
};


template<EncoderConf* encoderConfs,size_t count,typename pos_t>
struct EncoderManager{
    static void begin(){
        EncoderInitializer<encoderConfs,pos_t,count,count>::init();
    }
    template <size_t enc_id>
    static void fetchPosition(pos_t& pos){
        static_assert(enc_id<count,"id should be less than count");
        pos=RotaryEncoder_t<encoderConfs,pos_t,enc_id>::pos;
    }
};

} // namespace RotaryEncoder
