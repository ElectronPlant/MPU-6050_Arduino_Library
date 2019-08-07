#include "mpu_6050_library.h"

//            **************************
//            *      DEFINITIONS       *
//            **************************

//--- MPU ---
#define MPU_INTERRUPT_PIN 			2				// Interrupt pin in the Arduino Nano

#define TEST_LIMIT_TEST					10		// Just for testing. This is the number measurements that will be ignored

//            **************************
//            *      VARIABLES         *
//            **************************

//--- MPU ---
MpuDev test;  																			// Main MPU class object
extern volatile bool mpu_data_ready = false;				// Flag to check if there is new data available (true = new data ready)
volatile unsigned long count = 0;										// This is just for testing to delay
extern volatile unsigned long time_buffer = 0;			// This is to get the timing correct
volatile bool overflow = false;											// This indicates if there has been an overflow with the data from the MPU

// Test
//unsigned long previous_time = 0;
int16_t ax[30];
int16_t ay[30];
int16_t az[30];
int16_t gx[30];
int16_t gy[30];
int16_t gz[30];
unsigned long time[30];
float temp_test[50];

unsigned long previous_time;


//            **************************
//            *      FUNCTIONS         *
//            **************************

void mpuDataReady() {
	/* MPU interrupt call-back function.
	 * It sets the "mpu_data_ready" flag high when called
	 
	count++;
	if (count >= TEST_LIMIT_TEST) {
		mpu_data_ready = true;
		count = 0;
	}
	*/
	
	/* This function is called every time the MPU interrupt on pin 2 is set 'high'
	 * During the interrupt routine, millis() won´t update so it will record the time when the interrupt was called
	 * This is better for this case since I only want to log that :)
	 */

	// Check if the previous MPU values have been updated
	if (mpu_data_ready) overflow = true;

	// Update time
	mpu_data_ready = true;
	time_buffer = millis();
	
}

void error(uint16_t error_code){
  /* 
   * This function manages the states of the robot. The states are:
   *    0   -->   no problem
   *    1   -->   MPU initialization error.
   *		2	-->	  MPU failed the self-test.
   *    3   -->   MPU need to be calibrated (this is not an error, but a state).
   *    4   -->   MPU calibration error.
   */
	Serial.print(F("ERROR: "));
	Serial.println(error_code);
	Serial.print(F("MPU Error Code: "));
	Serial.println(test.mpu_state_global);
	while(true){
		// Stop until the correct procedure is defined
	}
}


void setup() {
	//Serial.begin(115200);
	//Serial.println(F("Serial debug mode for the MPU initialized :)"));

	// Variables
  bool correct;         // This flag is used for the verification
  bool mpu_calibrated;  // This flag indicated if the MPU needs to be calibrated or not

  	// --- State serial debug mode ---
	#ifndef SERIAL_INIT_MPU  //Start serial only in debug mode
		//initialize serial interface
		Serial.begin(115200);
		Serial.println(F("Serial debug mode for the MPU initialized :)"));
	#endif

	//--------------------------------------------------
	// MPU
	//--------------------------------------------------

  // --- Configure the interrupt for the MPU ---
	pinMode(MPU_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), mpuDataReady, RISING);

	// --- Initialize the MPU 1 ---
	correct = test.initialize_1();
	if (!correct) error(test.mpu_state_global);  // The MPU couldn't be initialized
  Serial.println(correct);

	//--------------------------------------------------
	// Do other initialization here while the thermal stabilization is achieved
	//--------------------------------------------------
	Serial.println(F("Waiting......."));
	delay(5000);
	Serial.println(F("waiting done ;-;"));

	// --- Initialize the MPU 1 ---
	correct = test.initialize_2();
	if (!correct) error(test.mpu_state_global);  // The MPU couldn't be initialized

	Serial.print(F("Hello =)    "));
	Serial.println(test.mpu_state_global);

	// testing
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);    // turn the LED off

	Serial.println(F("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"));
	Serial.println("-------------------------------------------------------");
	mpu_data_ready = false;
}

/*
void loop() {
  // remember to print the value of the interrupt pin :)
  if (mpu_data_ready) {
  	// Get time
  	unsigned long difference = millis() - previous_time;
  	previous_time = millis();
  	//Serial.println(F("------------------------\nMPU DATA\n------------------------"));
  	//Serial.println("Time : " + String(difference));

  	// Get data
  	int16_t values[6];
  	//Serial.print(F("Measurements: "));
  	test.getParameter6(values);
  	for (int i = 0; i < 6; i++) {
  		if (i != 0) Serial.print(F("; "));
  		Serial.print(values[i]);
  	}
  	Serial.println(F(""));
  	//delayMicroseconds(100);
  	mpu_data_ready = false;
  }

  digitalWrite(13, mpu_data_ready);
}

void loop() {

	// Variables
	int16_t values[6];
	uint16_t captured_data;

	captured_data = 0;

	//digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  // Get data from the IMU
  while (captured_data < 30){
  	if (mpu_data_ready) {
  	
  	// Get time
  	//time_buffer = millis();
  	
  	// Get data
  	test.getParameter6(values); // This has to be done as close as possible the the time capture

  	// process data
  	ax[captured_data] = values[0];
  	ay[captured_data] = values[1];
  	az[captured_data] = values[2];
  	gx[captured_data] = values[3];
  	gy[captured_data] = values[4];
  	gz[captured_data] = values[5];

  	// Process time
  	time[captured_data] = time_buffer - previous_time;
  	previous_time = time_buffer;

  	// Reset
  	mpu_data_ready = false;
  	
  	// Increment the data count
  	captured_data++;

  	}
  	// don't do anything here -_- (need to measure if the code is fast enough)
  }

  if (overflow) digitalWrite(13, HIGH);

  //Print data
  for (uint16_t i = 0; i < captured_data; i++){
  	Serial.print(String(ax[i]));
  	Serial.print(F(";"));
  	Serial.print(String(ay[i]));
  	Serial.print(F(";"));
  	Serial.print(String(az[i]));
  	Serial.print(F(";"));
  	Serial.print(String(gx[i]));
  	Serial.print(F(";")); 
  	Serial.print(String(gy[i]));
  	Serial.print(F(";"));
  	Serial.print(String(gz[i]));
  	Serial.print(F(";"));
  	Serial.println(String(time[i]));
  }

  //digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(10);                       // wait for a second/100
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  overflow = false;
}
*/

void loop() {

	// Variables
	int16_t values[6];
	double *phy;
	double *phy_gyro, *phy_accel;
	uint16_t captured_data;
	unsigned long time_buffer2;

	captured_data = 0;

	//digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

	Serial.println(".");

	// --- Initialize measurements ---
	test.initializeMeasurements();

  // Get data from the IMU
  while (true) {
	  if (mpu_data_ready) {
	  	
	  	// Get time
	  	//time_buffer = millis();
	  	
	  	// Get data
	  	// test.getParameter6(values); // This has to be done as close as possible the the time capture

	  	if(test.mpu_state_global != MPU_CORRECT) error(10);

	  	time_buffer2 = time_buffer;

	  	// process data
	  	phy_gyro = test.testGyroEst(time_buffer2);
	  	phy_accel = test.testAccelEst(time_buffer2);
	  	phy = test.simplifiedKF(time_buffer2);

	  	if(test.mpu_state_global != MPU_CORRECT) error(10);

	  	// Reset
	  	mpu_data_ready = false;

	  	//Print data
	  	Serial.print(String(phy[0] * 180/M_PI));
	  	Serial.print(F(", "));
	  	Serial.print(String(phy[1] * 180/M_PI));
	  	Serial.print(F(", "));
	  	Serial.print(String(phy_gyro[0] * 180/M_PI));
	  	Serial.print(F(", "));
	  	Serial.print(String(phy_gyro[1] * 180/M_PI));
	  	Serial.print(F(", "));
	  	Serial.print(String(phy_accel[0] * 180/M_PI));
	  	Serial.print(F(", "));
	  	Serial.println(String(phy_accel[1] * 180/M_PI));
	  }
  	// don't do anything here -_- (need to measure if the code is fast enough)

  	if (overflow) digitalWrite(13, HIGH);
	  overflow = false;
	}
}
