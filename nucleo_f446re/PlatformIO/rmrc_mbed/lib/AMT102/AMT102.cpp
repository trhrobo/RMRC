#include "AMT102.h"

RotaryInc::RotaryInc(PinName pin_a, PinName pin_b,int mode):mode_(mode){
    measur_ = false;
    init(pin_a,pin_b);
}

RotaryInc::RotaryInc(PinName pin_a,PinName pin_b,double circumference,int resolution,int mode)
    :mode_(mode),resolution_(resolution),circumference_(circumference){
    measur_ = true;
    init(pin_a,pin_b);
}

void RotaryInc::init(PinName pinA,PinName pinB){
    reset();
    pin_a_ = new InterruptIn(pinA,PullUp);
    pin_b_ = new InterruptIn(pinB,PullUp);
    pin_a_->rise(callback(this,&RotaryInc::riseA));
        
    if(mode_ == 2){
        pin_a_->fall(callback(this,&RotaryInc::fallA));
    }else if(mode_ == 4){
        pin_a_->fall(callback(this,&RotaryInc::fallA));
        pin_b_->rise(callback(this,&RotaryInc::riseB));
        pin_b_->fall(callback(this,&RotaryInc::fallB));
    }else{
        mode_ = 1;
    }
}

void RotaryInc::zero(){
    timer_.stop();
    timer_.reset();
    start_frag_ = false;
    flag_ = false;
    last_[0] = pulse_;
    speed_ = 0;
    count_ = 0;
    sum_ = 0;
    now_ = 0;
}

void RotaryInc::calcu(){
    if(!start_frag_){
        timer_.start();
        start_frag_ = true;
        last_[0] = pulse_;
        pre_t_[0] = 0;
        count_ = 1;
    }else if(flag_){
        now_ = timer_.read();
        timer_.reset();
        sum_ -= pre_t_[count_];
        pre_t_[count_] = now_;
        sum_ += now_;
        speed_ = (double)(pulse_ - last_[count_]) / sum_;
        last_[count_] = pulse_;
        if(count_ < 19){
            count_++;
        }else{
            count_ = 0;
        }
    }else{
        now_ = timer_.read();
        timer_.reset();
        pre_t_[count_] = now_;
        sum_ += now_;
        speed_ = (double)(pulse_ - last_[0]) / sum_;
        last_[count_] = pulse_;
        count_++;
        if(count_ > 19){
            count_ = 0;
            flag_ = true;
        }
    }
}

void RotaryInc::riseA(){
    pin_b_->read() ? pulse_-- : pulse_++;
    if(measur_){
        calcu();
    }
}

void RotaryInc::fallA(){
    pin_b_->read() ? pulse_++ : pulse_--;
    if(measur_){
        calcu();
    }
}

void RotaryInc::riseB(){
    pin_a_->read() ? pulse_++ : pulse_--;
    if(measur_){
        calcu();
    }
}

void RotaryInc::fallB(){
    pin_a_->read() ? pulse_-- : pulse_++;
    if(measur_){
        calcu();
    }
}

long long RotaryInc::get(){
    return pulse_;
}

double RotaryInc::getSpeed(){
    if(!measur_){
        return 0;
    }
    if(timer_.read_ms() > 150){
        zero();
    }
    return speed_ / resolution_ / mode_ * circumference_;
}

int RotaryInc::diff(){
    int diff = pulse_ - prev_;
    prev_ = pulse_;
    return diff;
}
    
void RotaryInc::reset(){
    pulse_ = 0;
    prev_ = 0;
    if(measur_){
        zero();
    }
}

RotaryInc::~RotaryInc(){
    pin_a_->disable_irq();
    pin_b_->disable_irq();
    delete pin_a_;
    delete pin_b_;
}