
#define EXT_MOTOR_4KHZ 



/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
	/****************  Specific PWM Timers & Registers for the MEGA's   *******************/
	/******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
	#if !(NUMBER_MOTOR == 4)
	#error "only 4 motors allowed"
	#endif
	
	#if defined(MEGA)//
	#error "only arduino 328"
	#endif

	#if defined(PROMINI)
	#if defined(EXT_MOTOR_32KHZ)
	OCR1A = (motor[0] - 1000) >> 2; //  pin 9
	OCR1B = (motor[1] - 1000) >> 2; //  pin 10
	OCR2A = (motor[2] - 1000) >> 2; //  pin 11
	OCR2B = (motor[3] - 1000) >> 2; //  pin 3
	#elif defined(EXT_MOTOR_4KHZ)
	OCR1A = (motor[0] - 1000) << 1; //  pin 9
	OCR1B = (motor[1] - 1000) << 1; //  pin 10
	OCR2A = (motor[2] - 1000) >> 2; //  pin 11
	OCR2B = (motor[3] - 1000) >> 2; //  pin 3
	#elif defined(EXT_MOTOR_1KHZ)
	OCR1A = (motor[0] - 1000) << 3; //  pin 9
	OCR1B = (motor[1] - 1000) << 3; //  pin 10
	OCR2A = (motor[2] - 1000) >> 2; //  pin 11
	OCR2B = (motor[3] - 1000) >> 2; //  pin 3
	#else
	#error only 32khz or 4khz or 1khz on 328 device
	#endif
	#endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc)
{   // Sends commands to all motors
	for (uint8_t i =0;i<NUMBER_MOTOR;i++) motor[i]=mc;
	writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
	/****************            mark all PWM pins as Output             ******************/
	for(uint8_t i=0;i<NUMBER_MOTOR;i++) pinMode(PWM_PIN[i],OUTPUT);
	
	#if defined(PROMINI)
	TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
	TCCR1B = (1<<WGM13) | (1<<CS10);
	#if defined(EXT_MOTOR_32KHZ)
	ICR1   = 0x00FF; // TOP to 255;
	TCCR2B =  (1<<CS20);
	#elif defined(EXT_MOTOR_4KHZ)
	ICR1   = 0x07F8; // TOP to 1023;
	TCCR2B =  (1<<CS21);
	#elif defined(EXT_MOTOR_1KHZ)
	ICR1   = 0x1FE0; // TOP to 8184;
	TCCR2B =  (1<<CS20) | (1<<CS21);
	#else
	#error only 32khz or 4khz or 1khz on 328 device
	#endif
	TCCR1A |= _BV(COM1A1); // connect pin  9 to timer 1 channel A
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
	TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
	TCCR2A |= _BV(COM2B1); // connect pin  3 to timer 2 channel B
	#endif
	
	writeAllMotors(MINCOMMAND);
	delay(300);
}

#if defined(SERVO)
#error "*** no servos in pocketquad"
#endif


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
	int16_t maxMotor;
	uint8_t i;

	#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

	#if NUMBER_MOTOR > 3
	//prevent "yaw jump" during yaw correction
	axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
	#endif
	/****************                   main Mix Table                ******************/
	#ifdef MY_PRIVATE_MIXING
	#include MY_PRIVATE_MIXING
	#else
	#ifdef QUADP
	motor[0] = PIDMIX( 0,+1,-1); //REAR
	motor[1] = PIDMIX(-1, 0,+1); //RIGHT
	motor[2] = PIDMIX(+1, 0,+1); //LEFT
	motor[3] = PIDMIX( 0,-1,-1); //FRONT
	#endif
	#ifdef QUADX
	motor[0] = PIDMIX(-1,+1,-1); //REAR_R
	motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
	motor[2] = PIDMIX(+1,+1,+1); //REAR_L
	motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
	#endif
	#ifdef Y4
	motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
	motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
	motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
	motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
	#endif
	#ifdef VTAIL4
	motor[0] = PIDMIX(+0,+1, +1); //REAR_R
	motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
	motor[2] = PIDMIX(+0,+1, -1); //REAR_L
	motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
	#endif
	#endif //MY_PRIVATE_MIXING

	/****************                normalize the Motors values                ******************/
	#ifdef LEAVE_HEADROOM_FOR_MOTORS
	// limit this leaving room for corrections to the first #n of all motors
	maxMotor=motor[0];
	for(i=1; i < LEAVE_HEADROOM_FOR_MOTORS; i++)
	if (motor[i]>maxMotor) maxMotor=motor[i];
	if (maxMotor > MAXTHROTTLE) { // this is a way to still have good gyro corrections if at least one motor reaches its max.
		for(i=0; i < LEAVE_HEADROOM_FOR_MOTORS; i++)
		motor[i] -= maxMotor - MAXTHROTTLE;
	}
	for (i = 0; i < NUMBER_MOTOR; i++) {
		motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
		#if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
		if (rcData[THROTTLE] < MINCHECK)
		#else
		if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
		#endif
		#ifndef MOTOR_STOP
		motor[i] = MINTHROTTLE;
		#else
		motor[i] = MINCOMMAND;
		#endif
		if (!f.ARMED)
		motor[i] = MINCOMMAND;
	}
	#else // LEAVE_HEADROOM_FOR_MOTORS
	maxMotor=motor[0];
	for(i=1; i< NUMBER_MOTOR; i++)
	if (motor[i]>maxMotor) maxMotor=motor[i];
	for(i=0; i< NUMBER_MOTOR; i++) {
		if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
		motor[i] -= maxMotor - MAXTHROTTLE;
		motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
		#if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
		if (rcData[THROTTLE] < MINCHECK)
		#else
		if (rcData[THROTTLE] < MINCHECK)
		#endif
		#ifndef MOTOR_STOP
		motor[i] = MINTHROTTLE;
		#else
		motor[i] = MINCOMMAND;
		#endif
		if (!f.ARMED)
		motor[i] = MINCOMMAND;
	}
	#endif // LEAVE_HEADROOM_FOR_MOTORS
}