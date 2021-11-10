// packages 
// motor control
#include <Servo.h>

/*#######################################################################################*/
// definition of constants and conversions
// motor control
const int MIN_PULSE_LENGTH = 1000;	// Minimum pulse length in µs
const int MAX_PULSE_LENGTH = 2000; 	// Maximum pulse length in µs

// Servo setup
Servo ESC0, ESC1, ESC2, ESC3;

// Finite state machine setup
static bool armed = false;

// Rx setup
volatile static int PPM[6];
bool initialized = false;
byte channel = 0;
volatile float PWM_start = 0;
volatile float PWM_stop = 0;
volatile float PWM_width = 0;
const int PWM_detection_thr = 4000;   //mu s
const byte RX_PIN = 2;    // on PD2 pin (INT0)  
const byte NB_CHANNELS = 6;

// mixing
const float gain = 0.2;   // gain that is multiplied for mixing resultant PWM signal
const int PWM_center = 1500;    // centered PWM pulse to subtract from roll, pitch yaw

/*#######################################################################################*/
// setup 
void setup()
{
	// Serial communication setup
	Serial.begin(9600);	// baud rate

	// Attach the pins to ESCs
	ESC0.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);	// on PD4 pin
	ESC1.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);	// on PD5 pin
	ESC2.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);	// on PD6 pin
	ESC3.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);	// on PD7 pin

	// Idle the engines at startup
	IdleAllESCs();
	Serial.println("All engines are idled");

	// finite state machine, (static: exists for entirety of program)
	armed = false;
	Serial.print("The initial state is: ");
  Serial.println(armed);
	

	// Receiver interrupt gets triggered for rising voltage, ISR = interrupt service routine
	attachInterrupt(digitalPinToInterrupt(RX_PIN), getWidth, RISING);	
}

/*#######################################################################################*/
// loop
void loop()
{
  /*
  Serial.print("PPM[0]: "); Serial.println(PPM[0]);
  Serial.print("PPM[1]: "); Serial.println(PPM[1]);
  Serial.print("PPM[2]: "); Serial.println(PPM[2]);
  Serial.print("PPM[3]: "); Serial.println(PPM[3]);
  Serial.print("PPM[4]: "); Serial.println(PPM[4]);
  Serial.print("PPM[5]: "); Serial.println(PPM[5]);
  

  Serial.print("ESC0: "); Serial.println(PWM_ESC0());
  Serial.print("ESC1: "); Serial.println(PWM_ESC1());
  Serial.print("ESC2: "); Serial.println(PWM_ESC2());
  Serial.print("ESC3: "); Serial.println(PWM_ESC3());
  delay(500);
  */
  
	if (PPM[4] > 1750)	// C switch is on (2nd notch)
	{
		armed = true;
		Serial.println("STATE MACHINE: ARMED "); 
	} else    // C switch is off or 1st notch
	{
		armed = false;
		Serial.println("STATE MACHINE: DISARMED "); 
	}

	if (armed == false)
	{
			IdleAllESCs();
	} else
  {
			PWM_write();
	}
}

/*#######################################################################################*/
// all other functions
// motor control
// IdleAllESCs 
void IdleAllESCs()
{
	ESC0.writeMicroseconds(MIN_PULSE_LENGTH);
	ESC1.writeMicroseconds(MIN_PULSE_LENGTH);
	ESC2.writeMicroseconds(MIN_PULSE_LENGTH);
	ESC3.writeMicroseconds(MIN_PULSE_LENGTH);
}

// PWM_write
//    X configuration:
//  ESC0(CCW)  ESC1
//         \  /
//         /  \
//     ESC3   ESC2(CCW)
/*roll: -PPM[0], pitch: PPM[1], throttle: PPM[2], yaw: PPM[3], 
  3 mode switch: PPM[4], 2 mode switch: PPM[5]*/

void PWM_write()
{
	ESC0.writeMicroseconds(PWM_ESC0());
	ESC1.writeMicroseconds(PWM_ESC1());

	ESC2.writeMicroseconds(PWM_ESC2());
	ESC3.writeMicroseconds(PWM_ESC3());
}

/* These ESC values are not necessarily constrained to 1000-2000 mu s*/
int PWM_ESC0()
{
  return (PPM[2] - centered(PPM[1])*gain + centered(PPM[0])*gain + centered(PPM[3])*gain);
}

int PWM_ESC1()
{
  return (PPM[2] - centered(PPM[1])*gain - centered(PPM[0])*gain - centered(PPM[3])*gain);
}

int PWM_ESC2()
{
  return (PPM[2] + centered(PPM[1])*gain - centered(PPM[0])*gain + centered(PPM[3])*gain);
}

int PWM_ESC3()
{
  return (PPM[2] + centered(PPM[1])*gain + centered(PPM[0])*gain - centered(PPM[3])*gain);
}

int centered(int PPM)
{
  return (PPM - PWM_center);
}

// pulseWidth, ISR: no serial communication can work in this loop, volatile vars
// replace this code with better function to get pulse width per channel PPm-reader could be used
void getWidth(void)
{
  PWM_stop = micros();
  PWM_width = PWM_stop - PWM_start;
  PWM_start = PWM_stop;

  if (initialized)
  {
    if (channel < 6)
    {
      PPM[channel] = PWM_width; 
    }
  }  

  if (PWM_width > PWM_detection_thr)
  {
    channel = 0;
    initialized = true;
  } else if ((channel + 1) < 6)
  {
    channel++;
  }
}
