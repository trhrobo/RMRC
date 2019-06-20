namespace ros{
class MD{
public:
	void MD(int pin_A, int pin_B, int pin_pwm);
    void OUTPUT(int check, float pwm);
    void STOP(void);
private:
	int user_A, user_B, int user_pwm;
};
}

