#include<pigpiod_if2.h>

using ros::MD;

MD::MD(int pin_A, int pin_B, int pwm_pin){
	user_A = pin_A;
	user_B = pin_B;
	user_pwm = pwm_pin;
}

MD::OUTPUT(int check, int pwm){
//check == 1 front
//check == 0 back
	if(check){
		gpio_write(pi, user_A, 1);
		gpio_write(pi, user_B, 0);
	}else{
		
		gpio_write(pi, user_A, 0);
		gpio_write(pi, user_B, 1);
	}
	set_PWM_dutycycle(pi, pwm_pin, pwm);
	
}
