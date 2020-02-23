#ifndef ROTARY_INC_H
#define ROTARY_INC_H
#include "mbed.h"
#ifndef M_PI
#define M_PI 3.14159265358979
#endif

/*ロータリーエンコーダーを使う。
 *速度計測は、タイヤの円周とロリコンの分解能を指定した場合に有効になる。
 *RotaryInc(PinName pinA, PinName pinB,double circumference,int Resolution,int mode = 0);//速度計測有効
 *RotaryInc(PinName pinA, PinName pinB,int mode = 0);//速度計測無効
 */
//RotaryInc rotary(PA_1,PA_3,2 * 50.8 * M_PI,200);

class RotaryInc{
public:
    RotaryInc(PinName user_a, PinName user_b,double circumference,int resolution,int mode = 0);//速度計測有効
    RotaryInc(PinName user_a, PinName user_b,int mode = 0);//速度計測無効
    ~RotaryInc();
    long long get();
    double getSpeed();
    void reset(); 
    int diff();
private:
    InterruptIn *pin_a_,*pin_b_;
    Timer timer_;
    long long pulse_;
    long long last_[20];
    long long prev_;
    int count_;
    int mode_;
    int resolution_;
    double now_;
    double sum_;
    double pre_t_[20];
    double speed_;
    double circumference_;
    bool measur_;
    bool start_frag_;
    bool flag_;
    void init(PinName,PinName);
    void riseA(void);
    void riseB(void);
    void fallA(void);
    void fallB(void);
    void calcu(void);
    void zero(void);
};

#endif /* ROTARY_INC_H */