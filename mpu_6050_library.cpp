/*
 *                     ********************
 *                     *    MPU Library   *
 *                     ********************
 *
 *  :::::::::::::::::::
 *  ::::Source File::::
 *  :::::::::::::::::::
 *  Written by: David Arnaiz
 *  Github: TBD
 *
 *  Released under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
 *  license (https://creativecommons.org/licenses/by-sa/4.0/)
 *
 *  ---------------
 *  - MPU Library -
 *  ---------------
 * This library will define the high level functions for the MPU6050 gyroscope and Accelerometer so the rest of the code is transparent to it. 
 * This includes:
 *     +) Initialization.
 *     +) Self-test.
 *     +) Calibration.
 *     +) Retrieving the data from the sensor.
 * 
 *  -------------------
 *  - Acknowledgments -
 *  -------------------
 *  This file uses the I2Cdev library made by "jrowberg" which is available on Githib: https://github.com/jrowberg/i2cdevlib or on https://www.i2cdevlib.com
 *  (It uses millis() for the timeout --> check if the timers are modified)
 *  This library was inspired by the mpu_6050_library also implemented by "jrowberg" which is available on Github: 
 *  https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 *  The self-test procedure was based on the one proposed by "kriswiner" which is also available on Github: 
 *  https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino
 *
 *  --------------
 *  - Change Log -
 *  --------------
 * V0.0 27/12/2018 First issue.
 * V1.0 03/02/2019 Second issue.
 *     -) Approximated EKF implemented.
 *     -) Removed redundancies.
 *     -) Updated initialization code.
 */

#include "mpu_6050_library.h"
#include "I2Cdev.h"

// External variables:
extern volatile bool mpu_data_ready;              // This variable is used as an interrupt
extern volatile unsigned long time_buffer;        // This is to get the timing correct


//            **************************
//            *      TOOLS - OTHER     *
//            **************************

bool MpuDev::readMpuRegisters(uint8_t address_funct, uint8_t *buffer_funct, uint8_t length_funct) {
  /* This function reads consecutive registers of the MPU. 
   * The first register is set by address_funct and the number of registers read by length. The values will be stored 
   * in the byte array given by *buffer_funct.
   * 
   * The I2C interface will be done as follows:
   *      1) The communication will be established normally.
   *      2) If there is a timeout, the communication will be tried again I2C_MPU_RETRIES times.
   *      3) If the communication keeps failing it will raise an error.
   * 
   * Parameters:
   *      @param address_funct      --> (uint8_t) Address of the first register.
   *      @param *buffer_funct      --> (uint8_t) Pointer to a byte array where the data will be stored.
   *      @param length_funct       --> (uint8_t) Length of *buffer_funct and number of registers that will be read.
   *      @return status            --> (bool) State of the register reading process (true = success)
   */

  // --- Loop for the retires ---
  for (uint8_t i = 0; i < I2C_MPU_RETRIES; i++) {
    // -- Check if the communication is correct --
    if (I2Cdev::readBytes(I2C_ADDRESS_MPU, address_funct, length_funct, buffer_funct, I2C_TIMEOUT_CON) == length_funct) return true;
  }

  // -- Communication error --
  mpu_state_global = MPU_I2C_ERROR;  // Change state of the MPU to error
  for (uint8_t i = 0; i < length_funct; i++) {  // Set the output to zero
    *(buffer_funct + i) = 0;
  }

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("I2C_Error (*.*) reading"));
  #endif

  return false;
}

bool MpuDev::readMpuRegister(uint8_t address_funct, uint8_t *buffer_funct) {
  /* This function reads 1 register of the MPU. 
   * 
   * Parameters:
   *      @param address_funct      --> (uint8_t) Address of the register.
   *      @param *buffer_funct      --> (uint8_t) Pointer to a byte where the information will be loaded.
   *      @return status            --> (bool) State of the register reading process (true = success)
   */
  return readMpuRegisters(address_funct, buffer_funct, 1);
}

bool MpuDev::readMpuData(uint8_t address_funct, int16_t *data_funct) {
  /* This function reads a 16 bit value from the MPU. 
   * This is used to read a single gyroscope, accelerometer or temperature measurement.
   * 
   * Parameters:
   *      @param address_funct      --> (uint8_t) Address of the register.
   *      @param *buffer_funct      --> (uint16_t) Pointer a int16_t variable where the data will be loaded.
   *      @return status            --> (bool) State of the register reading process (true = success)
   */

  uint8_t buffer_funct[2];         // This variable is to holds the raw data from the registers
  bool communication_successful;  // This is used to know is the communication is correct or not
  
  // --- Read data form the device ---
  communication_successful = readMpuRegisters(address_funct, buffer_funct, 2);

  // --- Convert data ---
  *data_funct = (buffer_funct[0] << 8) | (buffer_funct[1]);

  return communication_successful;
}

bool MpuDev::readMpuMeasurements(int16_t *data_funct) {
  /* This function reads the accelerometer and gyroscope measurements.
   * To take advantage in the consecutive register reading, the temperature will also be read, then ignored.
   * 
   * Parameters:
   *      @param *buffer_funct      --> (uint16_t) Pointer a int16_t array to store the measurements (6 in total).
   */

  uint8_t buffer_funct[14];         // This variable is to holds the raw data from the registers
  bool communication_successful;   // This is used to know is the communication is correct or not
  
  // --- Read data form the device ---
  communication_successful = readMpuRegisters(MPU_ACCEL_REG_BASE, buffer_funct, 14);  // Communication failed

  // --- Convert data ---
  data_funct[0] = (buffer_funct[ 0] << 8) | (buffer_funct[ 1]);
  data_funct[1] = (buffer_funct[ 2] << 8) | (buffer_funct[ 3]);
  data_funct[2] = (buffer_funct[ 4] << 8) | (buffer_funct[ 5]);
  data_funct[3] = (buffer_funct[ 8] << 8) | (buffer_funct[ 9]);
  data_funct[4] = (buffer_funct[10] << 8) | (buffer_funct[11]);
  data_funct[5] = (buffer_funct[12] << 8) | (buffer_funct[13]);

  return communication_successful;
}

bool MpuDev::writeMpuRegister(uint8_t address_funct, uint8_t buffer_funct, bool check_funct = true) {
  /* This function writes one register of the MPU. 
   * The register address is set by address_funct and the value by buffer_funct 
   * 
   * The I2C interface will be done as follows:
   *      1) Write the MPU register.
   *      2) Read the MPU register. This can be disabled by setting check_funct = false
   *      2) If there is a timeout or the read value is not correct, the communication will be retried again I2C_MPU_RETRIES times.
   *      3) If the communication keeps failing it will raise an error.
   * 
   * Parameters:
   *      @param address_funct      --> (uint8_t) Address of the register.
   *      @param buffer_funct       --> (uint8_t) Value of the register to be written
   *      @param check_funct        --) (bool) It is set to true by default, but if set to false it will skip the verification process
   *      @return status            --> (bool) State of the register reading process (true = success)
   */

  // --- Loop for the retires ---
  for (uint8_t i = 0; i <= I2C_MPU_RETRIES; i++) {
    
    // -- write --
    if (!I2Cdev::writeBytes(I2C_ADDRESS_MPU, address_funct, 1, &buffer_funct)) continue;
    // Writing is correct
    
    if (check_funct) {
      // -- Read --
      uint8_t buffer_funct2;
      if (!readMpuRegister(address_funct, &buffer_funct2)) continue;

      // -- Check the values --
      if (buffer_funct2 == buffer_funct) return true;
    } else {
      return true;
    }
  }

  // -- Communication error --
  mpu_state_global = MPU_I2C_ERROR;
  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("I2C_Error (*.*) writing"));
  #endif
  return false;
}



bool MpuDev::updateMpuRegister(uint8_t address_funct, uint8_t values_funct, uint8_t mask_funct, bool check_funct = true) {
  /* This function is to only changes the given values in the register.
   * This function will access the register set by "address_funct" and only update the bits set as '1' in "mask_funct" to the 
   * value on "values_funct"
   * Parameters:
   *      @param address_funct      --> (uint8_t) Address of the register to be updated 
   *      @param values_funct       --> (uint8_t) Byte with the desired values of the register (the ones not set in the mask will be ignored)
   *      @param mask_funct         --> (uint8_t) Byte indicating which bits will be modified (if '1' it will be modified)
   *      @param check_funct        --) (bool) It is set to true by default, but if set to false it will skip the verification process
   *      @return status            --> (bool) State of the register update process (true = success)
   */

  uint8_t current_values;

  // --- Read the register ---
  if (!readMpuRegister(address_funct, &current_values)) return false;

  // --- calculate the new values ---
  current_values = (values_funct & mask_funct) | (current_values & ~mask_funct);

  // --- write the values ---
  if (!writeMpuRegister(address_funct, current_values, check_funct)) return false;

  return true;
}


//            **************************
//            *     CONFIGURATION      *
//            **************************

bool MpuDev::initialize_1() {
	/* This function initializes the MPU.
	 * It will initialize the I2C communication, wake up the device, perform the self-test, configure the MPU and loads the calibration data
   * from the EEPROM if it is available.
   * This function should be called as soon as possible as this will turn on the MPU so it starts reaching the thermal stabilization.
   * Once thermal stabilization has been achieved, intialization_2 should be called to finish configuring the device. 
   *
   * Parameters:
   *      @return status      --> (bool) Status of the MPU (true = correct, false = error) 
	 */

  //--------------------------------------------------
  // System initialization
  //--------------------------------------------------
  // This is only needed if it is not being done elsewhere

	// -- Start I2C --
	#ifdef I2C_CONFIGURE_MPU
		// Configure the I2C bus
		Wire.begin();
		Wire.setClock(I2C_CLK_SPEED); // I2C 400kHz fast mode
	#endif

	// --- State serial debug mode ---
	#ifdef SERIAL_INIT_MPU  //Start serial only in debug mode
		//initialize serial interface
		Serial.begin(SERIAL_SPEED);
		Serial.println(F("Serial debug mode for the MPU initialized :)"));
	#endif

	//--------------------------------------------------
  // Device I2C communication
  //--------------------------------------------------

  // --- Check communication ---
  // The communication is checked by reading the MPU ID

  uint8_t buffer_funct; // This is to hold the register value
  
  // -- Read ID --
  if (!readMpuRegister(MPU_DEVICE_ID_REG, &buffer_funct)){
    
    // debug mode
    #ifdef DEBUG_MODE_MPU
      Serial.println(F("I2C communication failed....."));
    #endif

    mpu_state_global = MPU_I2C_ERROR;  // Set the error state
    return false;
  }

  // -- Check ID --
  if ((buffer_funct & 0x7E) != MPU_DEVICE_ID_VALUE) {
    
    // debug mode
    #ifdef DEBUG_MODE_MPU
      Serial.println(F("MPU ID incorrect -_-"));
    #endif

    mpu_state_global = MPU_I2C_ERROR;  // Set the error state
    return false;
  }


  //--------------------------------------------------
  // Start MPU
  //--------------------------------------------------

  // --- Set the clock ---
  // The clock reference will be set the the gyro-Z
  updateMpuRegister(MPU_CLOCK_REF_ADDR, MPU_CLOCK_ZGYRO, MPU_CLOCK_REF_MASK);

  // --- Wakeup device ---
  setLowPowerMode(false);

  //--------------------------------------------------
  // Self-test
  //--------------------------------------------------

  if (!checkMpu()){
    
    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.println(F("Self-test failed ºuº"));
    #endif

    return false; 
  }

  //--------------------------------------------------
  // Configure MPU
  //--------------------------------------------------

  configureMpu();
  if (mpu_state_global == MPU_I2C_ERROR) return false;

  //--------------------------------------------------
  // Check calibration
  //--------------------------------------------------

  if (!checkCalibration()) {
    if (mpu_state_global == MPU_I2C_ERROR) return false;
  }

  // Debug
	#ifdef DEBUG_MODE_MPU
			Serial.println(F("Device Initialized successfully"));
	#endif
  
  return true;
}

bool MpuDev::initialize_2() {
  /* This function makes the calibration if needed and calculates the offset_correction values.
   * This function should be called once the thermal stabilization has been reached (which implies that initialize_1 has been called earlier).
   * The thermal stabilization point would be reached 5 minutes after the MPU has been turned on.
   * initialize_1 and initialize_2 have been divided so additional code can be executed while waiting for the thermal stabilization.
   * 
   * Parameters:
   *      @return bool    --> (bool) state of the initialization, true = success
   */

  bool correct_funct = true;

  // --- Calibrate the MPU ---
  
  if (mpu_state_global == MPU_NOT_CALIBRATED) {   // Check is calibration is required
    
    // -- Configure MPU for calibration --
    changeFullScale(MPU_DEFAULT_ACCEL_REG_VALUE, MPU_DEFAULT_GYRO_REG_VALUE);            // Sets the full-scale to the most sensitive one
    updateMpuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_DEFAULT, MPU_DLPF_MASK);         // Set the low pass filter. Check .h for more info
    I2Cdev::writeByte(I2C_ADDRESS_MPU, MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_DEFAULT);   // Sets the sample rate. Check .h for more info
    resetSignalPath();                                                                   // Resets the signal path of the MPU
    if (mpu_state_global == MPU_I2C_ERROR) return false;
    
    // -- Calibrate the MPU --
    correct_funct = performCalibration();
    if (!correct_funct) return false;

    // -- Configure MPU --
    configureMpu();
  }

  // --- Get offset correction ---

  // -- Set the sampling rate --
  #ifdef FAST_CALIBRATION_CORRECTION
    updateMpuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_DEFAULT, MPU_DLPF_MASK);         // Set the low pass filter. Check .h for more info 
                                                                                         // (needs to be changed also to avoid issues)
    I2Cdev::writeByte(I2C_ADDRESS_MPU, MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_DEFAULT);   // Sets the sample rate. Check .h for more info
  #endif
  
  // -- Reset the signal path --
  resetSignalPath();
  if (mpu_state_global == MPU_I2C_ERROR) return false;

  // -- Get values --
  getOffsetCorrection();

  // -- Re-configure the system --
   #ifdef FAST_CALIBRATION_CORRECTION
    configureMpu();
    // -- Reset the signal path --
    resetSignalPath();
  #endif

  // -- Check --
  if (mpu_state_global != MPU_NOT_INITIALIZED) {

    #ifdef DEBUG_MODE_MPU
      Serial.println("The MPU couldn't be initialized correctly. ´:(");
    #endif

    return false;
  }
  
  // MPU Initialized correctly :p 
  mpu_state_global = MPU_CORRECT;
  return true;
}

bool MpuDev::checkMpu() {
  /* This function checks the gyro and the accelerometer to check if they are damaged.
   * This procedure is inspired by "https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino".
   * If this fails it means that the device is damaged and should be replaced :(
   * 
   * Parameters:
   *      @return bool      --> (bool) Status of the MPU (true = correct, false = error)
   */

  // Variables
  uint8_t values_raw[3];  // Register values with the self-test results
  uint8_t self_test;      // Self-test value (they are 5 bit unsigned integers)
  float factory_trimmed;   // factory_trimmed value
  float result;           // Result of the test

  // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.println("::Self-Test::");
    #endif

  // --- Set the self-test of the MPU ---
  // This sets the self-test bits on the accel to '1' and the full-scale range to +-8g
  // This sets the self-test bits on the gyro to '1' and the full-scale range to 250dps 
  changeFullScale(MPU_SELF_TEST_ACCEL_REG_VALUE, MPU_SELF_TEST_GYRO_REG_VALUE);

  // --- Wait for the self-test to be completed ---
  delay(MPU_SELF_TEST_WAIT_TIME);

  // --- Get the values ---
  if (!readMpuRegisters(MPU_SELF_TEST_RESULT_ADDR_BASE, values_raw, 3)) return false;

  // --- Restore the full-scale range ---
  // This is done now since the function will end if any error is detected
  // (No need to do this anymore, it will be done later :)
  // changeFullScale(MPU_DEFAULT_ACCEL_REG_VALUE, MPU_DEFAULT_GYRO_REG_VALUE);

  // --- Check accelerometer ---

   for (uint8_t i = 0; i < 3; i++) {
    
    // -- Get the self-test result --
    self_test = (values_raw[i] >> 3) | (values_raw[3] & (0x30 >> (2 * i)));

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(self_test);
      Serial.print(F(" --> "));
    #endif

    // -- Get the factory trimmed value
    factory_trimmed = (1392.64)*(pow( (0.92/0.34) , (((float)self_test - 1.0)/30.0)));

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(factory_trimmed);
      Serial.print(F(" --> "));
    #endif

    // -- Check self-test --
    factory_trimmed = 100 + 100 * ((float)self_test - factory_trimmed)/factory_trimmed;  // Calculate percentage
    factory_trimmed = abs(factory_trimmed);

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.println(factory_trimmed);
    #endif

    if (factory_trimmed > MPU_SELF_TEST_THRESHOLD) {

      // Debug
      #ifdef DEBUG_MODE_MPU
        Serial.print(F("Accelerometer damaged:"));
        if (i == 0) {
          Serial.println(F("A_X"));
        } else if ( i == 2) {
          Serial.println(F("A_Y"));
        } else {
          Serial.println(F("A_Z"));
        }
      #endif

      mpu_state_global = MPU_SELF_TEST_FAILED_BASE + i;  // Set the error state
      return false;
    } 
  }

  // --- Check gyroscope ---

  for (uint8_t i = 0; i < 3; i++) {
    // -- Get the self-test result --
    self_test = values_raw[i] & 0x1F;  // Get the first 5 bits --> it is the 5 bit unsigned int result

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(self_test);
      Serial.print(F(" --> "));
    #endif

    // -- Get the factory trimmed value
    factory_trimmed = (3275.0)*(pow( 1.046 , ((float)self_test - 1.0) ));
    if (i == 1) factory_trimmed *= -1;

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(factory_trimmed);
      Serial.print(F(" --> "));
    #endif

    // -- Check self-test --
    factory_trimmed = 100 + 100 * ((float)self_test - factory_trimmed)/factory_trimmed;  // Calculate percentage
    factory_trimmed = abs(factory_trimmed);

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.println(factory_trimmed);
    #endif

    if (factory_trimmed > MPU_SELF_TEST_THRESHOLD) {

      // Debug
      #ifdef DEBUG_MODE_MPU
        Serial.print(F("Accelerometer damaged:"));
        if (i == 0) {
          Serial.println(F("G_X"));
        } else if ( i == 2) {
          Serial.println(F("G_Y"));
        } else {
          Serial.println(F("G_Z"));
        }
      #endif

      mpu_state_global = MPU_SELF_TEST_FAILED_BASE + 3 + i;  // Set the error state
      return false;
    } 
  }

  return true;
}

void MpuDev::configureMpu() {
  /*
   * This function updates the registers of the MPU to set the:
   *      -) Full-scale values for the gyro and accel.
   *      -) Digital low-pass filter.
   *      -) Interrupt output.
   *      -) Sample rate.
   * Then resets the signal path to avoid any issues with the data.
   * 
   * Parameters:
   *      NA        --> This function doesn't require or return any parameter
   */

  //--------------------------------------------------
  // Configure MPU
  //--------------------------------------------------

  // --- Configure the full-scale ---
  changeFullScale(MPU_ACCEL_CONFIG_VALUE, MPU_GYRO_CONFIG_VALUE);  // Sets the full-scale to the working one (It will be changed for calibration)
  
  // --- Set the low pass filter ---
  // The digital high-pass filter will limit the maximum sample frequency, check .h and the documentation of the MPU for more info. ----------------------------------------------------------- Needs adjustment
  updateMpuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_WORKING, MPU_DLPF_MASK); // Set the low pass filter. Check .h for more info

  // --- Configure the interrupt ---  
  // The interrupt will be set as DATA_RDY, so it will be set as high when all the measurement registers are updated (including temp, turn off?)
  updateMpuRegister(MPU_INTERRUPT_CONF_ADDR, MPU_INTERRUPT_DEFAULT, MPU_INTERRUPT_CONF_MASK);  // Check .h for more info

  // --- Set the sample rate ---
  // The maximum sample rate is 1kHz because of the accelerometer (it could go higher, but the accelerometer measurements will be repeated)
  // The sample rate will be set by MPU_SAMPLE_RATE, so the sample rate will be:
  //                          Sample rate = (Oscillation frequency)/(MPU_SAMPLE_RATE_DAFAULT + 1)
  I2Cdev::writeByte(I2C_ADDRESS_MPU, MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_WORKING);   // Check .h for more info

  //(Moved)
  //resetSignalPath();
}

void MpuDev::resetSignalPath() {
  /* This function resets the signal path of the MPU.
   *
   * Parameters:
   *      NA        --> This function doesn't require or return any parameter
   */

  // --- Reset signal path ---
  updateMpuRegister(MPU_RESET_SIGNAL_PATH_ADDR, MPU_RESET_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATH_MASK, false);

  delay(MPU_RESET_SIGNAL_PATH_DELAY);  // wait for a bit -------------------------------------------------------------------------------------------------------------------------uses delay
}

void MpuDev::changeFullScale(uint8_t accel_reg, uint8_t gyro_reg) {
  /* This function is to set the configuration register for the accelerometer and the gyroscope to the given values.
   * This registers are used to set the self-test (three most significant bits) on and to set the full-scale range (bit 4 and 3). 
   * The rest of the bits should be set as '0' (they are really undefined). The values of the full-scale range are:
   *                    Accelerometer                                     Gyroscope                       input
   *                    ------------                                      ---------                       ------
   *                    0 --> +-2g                                        0 --> +-250dps                  0x00
   *                    1 --> +-4g                                        1 --> +-500dps                  0x08
   *                    2 --> +-8g                                        2 --> +-1000dps                 0x10
   *                    3 --> +-16g                                       3 --> +-2000dps                 0x18
   * Parameters:
   *      @param accel_reg      --> (uint8_t) Desired value for the accelerometer configuration register
   *      @param gyro_reg       --> (uint8_t) Desired value for the gyroscope configuration register
   */ 

  // --- Set the accelerometer ---
  //I2Cdev::writeByte(I2C_ADDRESS_MPU, MPU_ACCELEROMETER_CONF_ADDR, accel_reg);
  updateMpuRegister(MPU_ACCELEROMETER_CONF_ADDR, accel_reg, MPU_ACCEL_CONFIG_MASK_VALUE);

  // --- Set the gyroscope ---
  //I2Cdev::writeByte(I2C_ADDRESS_MPU, MPU_GYRO_CONF_ADDR, gyro_reg);
  updateMpuRegister(MPU_GYRO_CONF_ADDR, gyro_reg, MPU_GYRO_CONFIG_MASK_VALUE);

  // Done :)
}

void MpuDev::setLowPowerMode(bool sleep_enabled) {
  /* This function sets the MPU in low power mode.
   * It will only set the MPU into sleep mode. It can be configured to wake up at a certain rate to get accelerometer values or 
   * to disable one of the accelerometers or gyroscopes to save energy.
   * Parameters:
   *      @param sleep_enabled      --> (boolean) This is value is used to enabled or disable the sleep mode (true = set sleep mode)
   */

  // --- Set the register ---
  if (sleep_enabled)
    updateMpuRegister(MPU_LOW_POWER_MODE_ADDR, MPU_LOW_POWER_MODE_ENABLE, MPU_LOW_POWER_MODE_MASK);
  else
    updateMpuRegister(MPU_LOW_POWER_MODE_ADDR, MPU_LOW_POWER_MODE_DISABLE, MPU_LOW_POWER_MODE_MASK);
}


//            **************************
//            *      CALIBRATION       *
//            **************************

void MpuDev::setOffsets(int16_t *offsets_funct2) {
  /*
   * This function sets the offset registers in the MPU. The goal of this function is to aid the "main" calibration function.
   *      @param *offsets_funct2  --> Pointer to the offsets array which will be loaded to the MPU 
   */

  uint8_t buffer_funct[2];                                // This array will hold the two bytes that form the int16_t 
  uint8_t address_funct = MPU_ACCEL_OFFSETS_BASE_ADDR;    // Set the address to the accelerometer first
  
  for (uint8_t i = 0; i < 6; i++) {
    // --- Split the data into bytes ---
    *(int16_t *) buffer_funct = *(offsets_funct2 + i);

    // --- Change address ---
    // This is because there is set the base address for the gyro
    if (i == 3) address_funct = MPU_GYRO_OFFSETS_BASE_ADDDR;

    // --- Write the register ---
    writeMpuRegister(address_funct, buffer_funct[1]);
    address_funct++;
    writeMpuRegister(address_funct, buffer_funct[0]);
    address_funct++;
  }
}

void MpuDev::calculateAverages(int16_t *averages_funct, uint16_t number_of_iterations, int16_t *targets_funct) {
  /*
   * This function calculates the average values obtained with the current offset values of the MPU.
   * The average values will be updated in the averages array, so it will overwrite the previous ones
   *      @param *averages_funct            --> Pointer to the averages array
   *      @param number_of_iterations       --> Number of iterations to be done
   *      @param *targets_funct             --> Pointer to the target array
   */
  
  int32_t sum_average[6] = {0, 0, 0, 0, 0, 0};  // This is to avoid overflow :(

  // Zero the average values
  for (uint8_t index = 0; index < 6; index++) *(averages_funct + index) = 0;  // Maybe is better just to create a local new array as zero? :p

  // -------------------------------
  // --- Obtain the measurements ---
  // -------------------------------
  for (int16_t index = 0; index < number_of_iterations; index++) {
    
    int16_t values_raw[6];  // Values read from the device
    
    while (!mpu_data_ready) {
      // Wait for the data to be ready
    }
    
      // -- Measure --
      getParameter6(values_raw);

      // -- Reset --
      mpu_data_ready = false;

      // -- Sum --
      for (uint8_t index = 0; index < 6; index++) sum_average[index] += values_raw[index];
  }

  // ------------------------------
  // --- Calculate the averages ---
  // ------------------------------
  for (uint8_t index = 0; index < 6; index++) {
    *(averages_funct + index) = (sum_average[index] / number_of_iterations) - *(targets_funct + index);
  }

  /*
  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("averages : "));
    // Serial.println(count);
    Serial.println(String(*(averages_funct + 0)));
    Serial.println(String(*(averages_funct + 1)));
    Serial.println(String(*(averages_funct + 2)));
    Serial.println(String(*(averages_funct + 3)));
    Serial.println(String(*(averages_funct + 4)));
    Serial.println(String(*(averages_funct + 5)));
  #endif
  */
}

bool MpuDev::calibrate(int16_t *offsets_funct) {
  /*
   *  This is the main function to calibrate the MPU. The calibration process goes as follows: 
   *    1)  The fist step is to find an offset value that produces a negative average (low offset) and other value 
   *        that produces a positive one (high offset). This values will be the initial range for the approximation
   *        process.
   *    2)  The next step is to follow a successive approximation process to decrease the offset range found during (1). 
   *        This is done by calculating the average offset between the low and high offsets and evaluating the average
   *        value. If the value is positive --> that offset will be the new high offset, or if negative, the value 
   *        will be the low offset. This will be repeated until the maximum number of iterations is done or the range
   *        is closed. 
   *    3)  The offsets register values (values which will be written to the registers) and the external offset values
   *        (error even after the offset register has been adjusted) are returned.
   *
   *         @param *offsets_funct          --> Pointer to the offsets array (this is done so the values can be
                                                loaded and stored in the EEPROM)
   *         @return bool                   --> Status of the calibration (true = success)
   */

  // Variables
  int16_t low_offsets[6], high_offsets[6];       // These arrays store the low and high offset values respectively
  int16_t low_values[6], high_values[6];
  int16_t averages[6];                           // Average array
  int16_t targets[] = {X_ACCEL_TARGET, Y_ACCEL_TARGET, Z_ACCEL_TARGET,
                       X_GYRO_TARGET, Y_GYRO_TARGET, Z_GYRO_TARGET};   // Target array, stores the expected values
  uint16_t count = 0;                            // This is to count the number of iterations 
  bool use_max_iterations = false;               // This is to set the maximum number of iterations for the average
  int16_t max_difference = 0;

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("MPU Calibration initialized :)"));
  #endif

  //--------------------------------------------------
  // Initialize
  //--------------------------------------------------
  for (uint8_t index = 0; index < 6; index++) {
    low_offsets[index]  = *(offsets_funct + index);
    high_offsets[index] = *(offsets_funct + index);
  }

  bool done = false;  // This is to know is the process has been completed

  //--------------------------------------------------
  // Locate initial range
  //--------------------------------------------------
  
  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Locating the initial range..."));
  #endif

  // Loop
  while (!done) {
    done = true;  // Set the process as done, then set it as not done if not

    // -- Locate the low offset value --
    setOffsets(low_offsets);                                            // Set the offsets
    calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets); // Calculate averages
    for (uint8_t index = 0; index < 6; index++) {
      low_values[index] = averages[index];
      if (averages[index] >= 0) {
        done = false;
        low_offsets[index] -= CALIBRATION_OFFSET_ADJUSTMENT;
      }
    }

    // -- Locate the high offset value --
    setOffsets(high_offsets);                                           // Set the offsets
    calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets); // Calculate averages
    for (uint8_t index = 0; index < 6; index++) {
      high_values[index] = averages[index];
      if (averages[index] <= 0) {
        done = false;
        high_offsets[index] += CALIBRATION_OFFSET_ADJUSTMENT;
      }
    }

    // Check if it is done or not 
    if (count++ > CALIBRATION_MAX_ITERATIONS) done = true;  // stop for over iterations
    // Check that there aren't are I2C issues
    if (mpu_state_global == MPU_I2C_ERROR) return false;

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(F("Iterations: "));
      Serial.println(count);
      Serial.println("[" + String(low_offsets[0]) + ", " + String(high_offsets[0]) + "] --> " + "[" + String(low_values[0]) + ", " + String(high_values[0]) + "]");
      Serial.println("[" + String(low_offsets[1]) + ", " + String(high_offsets[1]) + "] --> " + "[" + String(low_values[1]) + ", " + String(high_values[1]) + "]");
      Serial.println("[" + String(low_offsets[2]) + ", " + String(high_offsets[2]) + "] --> " + "[" + String(low_values[2]) + ", " + String(high_values[2]) + "]");
      Serial.println("[" + String(low_offsets[3]) + ", " + String(high_offsets[3]) + "] --> " + "[" + String(low_values[3]) + ", " + String(high_values[3]) + "]");
      Serial.println("[" + String(low_offsets[4]) + ", " + String(high_offsets[4]) + "] --> " + "[" + String(low_values[4]) + ", " + String(high_values[4]) + "]");
      Serial.println("[" + String(low_offsets[5]) + ", " + String(high_offsets[5]) + "] --> " + "[" + String(low_values[5]) + ", " + String(high_values[5]) + "]");
    #endif
  }  // Range found

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Done locating the initial range :p"));
    Serial.println(count);
    Serial.println(F("Reduce the range...."));
  #endif

  if (count < CALIBRATION_MAX_ITERATIONS) done = false;

  // -- calculate new offsets -- (it could be move outside the while loop and calculate the offsets is the other for loop)
  for(uint8_t index = 0; index < 6; index++) {
    *(offsets_funct + index) = (low_offsets[index] + high_offsets[index])/2;  // Set the new offset to the middle point 
  }

  //--------------------------------------------------
  // Decrease the initial range
  //--------------------------------------------------
  while (!done) {
    
    max_difference = 0;   // This is to store the maximum difference between the high and low offsets of all the components
    done = true;
    
    // -- Average with the new offset values --
    setOffsets(offsets_funct);
    if (use_max_iterations) {
      calculateAverages(averages, CALIBRATION_AVERAGES, targets);
    } else {
      calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets);
    }

    // Loop all the components
    for (uint8_t index = 0; index < 6; index++) {
      
      int16_t difference;  // This is to hold the current difference between the high and low offset for each components
      
      // -- Check if the new offsets are low or high --
      if (averages[index] <= 0) {  // Low offset
        low_offsets[index] = *(offsets_funct + index);
        low_values[index] = averages[index];
      } else {                      // High offset
        high_offsets[index] = *(offsets_funct + index);
        high_values[index] = averages[index];
      }

      // -- Evaluate --
      difference = high_offsets[index] - low_offsets[index];
      if (max_difference < difference) max_difference = difference;

      // -- Set the new offsets --
      *(offsets_funct + index) = (low_offsets[index] + high_offsets[index])/2;  // Set the new offset to the middle point for the next iteration
    }

    // Check the progress
    if (max_difference > CALIBRATION_MIN_ERROR) done = false;                     // Check if the calibration is done
    if (max_difference <= CALIBRATION_INITIAL_ERROR) use_max_iterations = true;   // Change the iterations value
    if (count++ > CALIBRATION_MAX_ITERATIONS) done = true;                        // stop for over iterations
    // Check that there aren't are I2C issues
    if (mpu_state_global == MPU_I2C_ERROR) return false;

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(F("Iterations: "));
      Serial.println(count);
      Serial.print(F("max_difference: "));
      Serial.println(max_difference);
      Serial.println("[" + String(low_offsets[0]) + ", " + String(high_offsets[0]) + "] --> " + "[" + String(low_values[0]) + ", " + String(high_values[0]) + "]");
      Serial.println("[" + String(low_offsets[1]) + ", " + String(high_offsets[1]) + "] --> " + "[" + String(low_values[1]) + ", " + String(high_values[1]) + "]");
      Serial.println("[" + String(low_offsets[2]) + ", " + String(high_offsets[2]) + "] --> " + "[" + String(low_values[2]) + ", " + String(high_values[2]) + "]");
      Serial.println("[" + String(low_offsets[3]) + ", " + String(high_offsets[3]) + "] --> " + "[" + String(low_values[3]) + ", " + String(high_values[3]) + "]");
      Serial.println("[" + String(low_offsets[4]) + ", " + String(high_offsets[4]) + "] --> " + "[" + String(low_values[4]) + ", " + String(high_values[4]) + "]");
      Serial.println("[" + String(low_offsets[5]) + ", " + String(high_offsets[5]) + "] --> " + "[" + String(low_values[5]) + ", " + String(high_values[5]) + "]");
    #endif
  }

  //--------------------------------------------------
  // Done
  //--------------------------------------------------

  // -- Select the best results --
  // The best will be the ones with the lowest absolute value average
  for (uint8_t index = 0; index < 6; index++) {
    if ((-1 * low_values[index]) <= high_values[index]) {
      *(offsets_funct + index) = low_offsets[index];
      *(offset_correction + index) = low_values[index];
    } else {
      *(offsets_funct + index) = high_offsets[index];
      *(offset_correction + index) = high_values[index];
    }
  }

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.print(F("Done locating the offsets XD\nNumber of iterations: "));
    Serial.println(count);
    Serial.println(String(*(offsets_funct))     + ", " + String(*(offsets_funct + 1)) + ", " + String(*(offsets_funct + 2)) + ", ");
    Serial.println(String(*(offsets_funct + 3)) + ", " + String(*(offsets_funct + 4)) + ", " + String(*(offsets_funct + 5)));
    Serial.println(String(*(offset_correction))     + ", " + String(*(offset_correction + 1)) + ", " + String(*(offset_correction + 2)) + ", ");
    Serial.println(String(*(offset_correction + 3)) + ", " + String(*(offset_correction + 4)) + ", " + String(*(offset_correction + 5)));
  #endif

  // The calibration is done, return true if correct and false is it stopped for over iterations
  if (count > CALIBRATION_MAX_ITERATIONS) {
    mpu_state_global = MPU_CALIBRATION_ERROR;
    return false;
  } 
  return true;
}

void MpuDev::getOffsetCorrection() {
  /*
   * This function obtains the offset correction values.
   * The offset correction values are used in case the calibration isn't able to zero the input or to compensate from small temperature differences
   * when the calibration data is loaded from the EEPROM. This function should only be called once the thermal stabilization of the MPU has been 
   * reached. Parameters:
   *      N/A       --> This function doesn't require or return any variable
   */

  int16_t targets_funct[] = {X_ACCEL_TARGET, Y_ACCEL_TARGET, Z_ACCEL_TARGET,
                             X_GYRO_TARGET, Y_GYRO_TARGET, Z_GYRO_TARGET};   // Target array, stores the expected values

  // Debug Mode
  #ifdef DEBUG_MODE_MPU
    Serial.print(F("Locating the offset correction values....."));
  #endif

  // --- Calculate the averages ---
  calculateAverages(CALIBRATION_CORRECTION_AVERAGES, offset_correction, targets_funct);

  // Debug Mode
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Completed!"));
    Serial.println(String(*(offset_correction))     + ", " + String(*(offset_correction + 1)) + ", " + String(*(offset_correction + 2)) + ", ");
    Serial.println(String(*(offset_correction + 3)) + ", " + String(*(offset_correction + 4)) + ", " + String(*(offset_correction + 5)));
  #endif
  // Done, the values are already stored in offset_correction

}

bool MpuDev::checkCalibration() {
  /*
   * This function check if the calibration is needed or not. If it doesn't need calibration it will load the offset
   * values to the MPU. But if it need to be calibrated, it will return false to wait until the temperature 
   * stabilization is achieved. Note that the system will need to be calibrated afterwards.
   *            @return bool                 --> Calibration status. If true, the system is already calibrated. 
   *                                             If false, the system will need to be calibrated 
   */

  // Variables
  int16_t mpu_offsets[6];               // Initial offset values
  float mpu_current_temperature;        // Temperature of the MPU
  float mpu_previous_temperature;       // Temperature of the MPU
  bool calibrated;                      // Flag to know if there is calibration data in the EEPROM  

  // --- Get current temperature ---
  mpu_current_temperature = getTemperature();

  // --- Check if there is data in the EEPROM ---
  calibrated = loadFromEEPROM(&mpu_previous_temperature, mpu_offsets);
  
  if (calibrated) {  // Calibration data exists
    float temp_difference = mpu_current_temperature - mpu_previous_temperature;  // Now this stores the temperature difference to avoid having an additional variable-----------------------------------------------------------Optimization could be done
    if (abs(temp_difference) < CALIBRATION_MAX_TEMP_DIFF) {  // The temperature is in range
      // No calibration needed, set the offsets and quit
      setOffsets(mpu_offsets); 
      // Check I2C
      if (mpu_state_global == MPU_I2C_ERROR) return false;

      // Debug
      #ifdef DEBUG_MODE_MPU
        Serial.println(F("Calibration data found. Loading data from EEPROM"));
      #endif

      return true;
    }
  }

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("No calibration data found. The MPU would need to be calibrated :("));
    Serial.println("Temperature: " + String(mpu_current_temperature));
  #endif

  // -- Set state --
  mpu_state_global = MPU_NOT_CALIBRATED;
  return false;  // leave the MPU running until while temperature stabilization is reached
}

bool MpuDev::performCalibration() {
  /*
   * This function does the calibration and stores the result in the EEPROM.
   * This function should only be executed if:
   *        1) check Calibration has returned false
   *        2) Enough time has passed since the MPU has been enabled so thermal stabilization has been achieved (5 min approx)
   *
   *      @return bool      --> Calibration result (true = calibration correct)
   */

  // Variables
  int16_t mpu_offsets[6] = {0, 0, 0, 0, 0, 0};   // Initial offset values
  float mpu_current_temperature;                // Temperature of the MPU
  bool correct;                                 // Flag to know if the calibration is correct or not

  // --- Start calibration ---
  correct = calibrate(mpu_offsets);  // Calibrate
  if (!correct) return false;  // calibration error

  mpu_state_global = MPU_NOT_INITIALIZED;

  // -- Get the calibration temperature --
  mpu_current_temperature = getTemperature();

  // -- Set the offset values --
  setOffsets(mpu_offsets);
  // Check
  if (mpu_state_global == MPU_I2C_ERROR) return false;

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Calibration done. Loading data to the EEPROM"));
  #endif

  // --- save to the EEPROM ---
  saveOnEEPROM(&mpu_current_temperature, mpu_offsets);

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("MPU calibrated and data loaded in the EEPROM"));
  #endif

  // Completed :)
    return true;
}


//            **************************
//            *        EEPROM          *
//            **************************


bool MpuDev::loadFromEEPROM(float *temperature_mpu, int16_t *offsets_funct) {
  /*
   * This function checks if there is any calibration data stored in the EEPROM and loads it.
   * The values are stored sequentially and the initial address is defined by MPU_EEPROM_OFFSET_ADDRESS.
   * The data is stored in the EEPROM as follows:
   *      -) MPU_CALIBRATION_CONTROL_BYTE:      Byte to confirm that there is data stored in the EEPROM.
   *      -) x_Accelerometer_offset_high:       Most significant byte of the X accelerometer offset value.
   *      -) x_accelerometer_offset_low:        Least significant byte of the X accelerometer offset value.
   *      -) y_Accelerometer_offset_high:       Most significant byte of the Y accelerometer offset value.
   *      -) y_accelerometer_offset_low:        Least significant byte of the Y accelerometer offset value.
   *      -) z_Accelerometer_offset_high:       Most significant byte of the Z accelerometer offset value.
   *      -) z_accelerometer_offset_low:        Least significant byte of the Z accelerometer offset value.
   *      -) x_gyroscope_offset_high:           Most significant byte of the X gyroscope offset value.
   *      -) x_gyroscope_offset_low:            Least significant byte of the X gyroscope offset value.
   *      -) y_gyroscope_offset_high:           Most significant byte of the Y gyroscope offset value.
   *      -) y_gyroscope_offset_low:            Least significant byte of the Y gyroscope offset value.
   *      -) z_gyroscope_offset_high:           Most significant byte of the Z gyroscope offset value.
   *      -) z_gyroscope_offset_low:            Least significant byte of the Z gyroscope offset value.
   *      -) temperature_high_high:         Most significant byte of the temperature at which the sensor was calibrated.
   *      -) temperature_high:              Second most significant byte of the temperature at which the sensor was calibrated.
   *      -) temperature_low:               Second least significant byte of the temperature at which the sensor was calibrated
   *      -) temperature_low_low:           Least significant byte of the temperature at which the sensor was calibrated.
   * 
   * The control byte is defined as follows: X7 X6 X5 X4 X3 X2 X1 X0
   *      -) X7, X6, X5, X4 --> 0xD if the data has been attempted to be stored
   *      -) X3, X2, X1, X0 --> 0xD if the data has been stored correctly
   * 
   * Parameters of the function:
   *      @param *temperature_mpu  -->  pointer to a float value to load the temperature form the EEPROM.
   *      @param *offsets_funct    -->  pointer to the offsets array where the values from the EEPROM will be loaded.
   *      @return bool             -->  boolean value set to "true" if the reading is correct, or "false" if not.
   */

  byte buffer_array[4];
  int address = MPU_EEPROM_OFFSET_ADDRESS;

  // --- Check if there is data stored in the EEPROM ---
  if (EEPROM.read(address) != 0xDD) {
    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(F("EEPROM not loaded. "));
      Serial.println(EEPROM.read(address));
    #endif
    return false; // No data, return false
  }

  // --- Read data from EEPROM ---
  address++;
    // Reset the temperature

  // -- Offsets --
  for (uint8_t i = 0; i < 6; i++) {
    
    // -- Get data --
    buffer_array[0] = EEPROM.read(address);
    address++;
    buffer_array[1] = EEPROM.read(address);
    address++;

    // convert
    *(offsets_funct + i) = *(int16_t *) buffer_array;
  }

  // -- Temperature --
  for (uint8_t i = 0; i < 4; i++) {
    buffer_array[i] = EEPROM.read(address);;
    address++;
  }
  *temperature_mpu = *(float *) buffer_array;
  
  // --- END ---
  return true;
}

void MpuDev::saveOnEEPROM(float *temperature_mpu, int16_t *offsets_funct) {
  /*
   * This function stores the calibration data in the EEPROM. 
   * The data structure is specified in the loadFromEEPROM function.
   * To ensure that the data isn't corrupted due to voltage spikes in the writing process, the data will be loaded first and
   * then read and compared. So it will be a relatively high time consuming process, but it only needs to be done once.
   *
   * * Parameters of the function:
   *      @param *temperature_mpu  -->  pointer to a float value to load the temperature form the EEPROM.
   *      @param *offsets_funct    -->  pointer to the offsets array where the values from the EEPROM will be loaded.
   */

  byte buffer_array[4];
  int address = MPU_EEPROM_OFFSET_ADDRESS;

  // --- Save the signature ---
  // At this point the least significant bits will be set to '0' and written to the expected value at the end of the process.
  // This is done to know if the data has been stored correctly even if the device turns off during the writing process. It 
  // must be also considered that the live of the EEPROM will be affected by this, since more writing cycles will be made, but
  // The calibration data should just be written one time in the live of the robot.

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Saving on the EEPROM..."));
    Serial.println("temperature: " + String(*temperature_mpu));
  #endif

  while (true) {
    EEPROM.update(address, 0xD0);
    // - Check -
    if (EEPROM.read(address) == 0xD0) break;
  }

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(EEPROM.read(address));
  #endif

  // --- Save the rest of the data ---
  address++;

  for (uint8_t i = 0; i < 7; i++) {
    
    // -- Split data into bytes --
    uint8_t data_length;
    if (i < 6) {  // Offsets
      *(int16_t *) buffer_array = *(offsets_funct + i);
      data_length = 2;
      //buffer_1 = (byte)(*(offsets_funct + i) >> 8);
    } else {       // Temperature
      *(float *) buffer_array = *temperature_mpu;
      //buffer_0 = *(temperature);
      //buffer_1 = (byte)(*(temperature) >> 8);
      data_length = 4;
    }

    // -- write data --
    for (uint8_t j = 0; j < data_length; j++) {
      while (true) {
        // Set data
        EEPROM.update(address, buffer_array[j]);
        // - Check -
        if (EEPROM.read(address) == buffer_array[j]) break;
      }
      address++;
    }

    // Debug
    #ifdef DEBUG_MODE_MPU
      Serial.print(F("."));
    #endif 
  }

  // --- Sign out ---
  // --- Save the signature ---
  while (true) {
    EEPROM.update(MPU_EEPROM_OFFSET_ADDRESS, 0xDD);
    // - Check -
    if (EEPROM.read(MPU_EEPROM_OFFSET_ADDRESS) == 0xDD) break;
  }

  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(EEPROM.read(MPU_EEPROM_OFFSET_ADDRESS));
  #endif 
}


//            **************************
//            *     Measurements       *
//            **************************

float MpuDev::getTemperature() {
  /*
   * This function just gets the temperature from the device. 
   * It uses float since double offers the same resolution in Arduino Nano and the rounding errors are not critical.
   * 
   * Parameters:
   *      @return float       --> (float) Temperature int degrees Celsius
   */

  int16_t temperature_temp;
  
  // --- Get data form the MPU ---
  readMpuData(MPU_TEMP_REG_BASE, &temperature_temp);

  // --- Convert value ---
  float foo_temp = ((float)(temperature_temp)/340) + 36.53;
  
  return foo_temp;
}

/*  Saved just in case

int16_t MpuDev::getTemperature() {
  /*
   * This function just gets the temperature from the device. 
   * It will be using int16_t and not float since this measurement isn't critical and this will reduce the used memory.
   * To reduce the truncation errors it will give the temperature in degrees Celsius * 1000
   * 
   *      @return int16_t       --> (int16_t) Temperature
   */ /*
  int16_t foo_temp = (((_mpuDevice.getTemperature())/340) * 1000) + 3653;
  return foo_temp;
}
*/

void MpuDev::getParameter6(int16_t *values_funct) {
  /* This function gets the raw accelerometer and gyroscope measurements.
   * The accelerometer values are in [m/s^2] and the angular speeds in [rad/s]
   * 
   * Parameters:
   *      @param *values_funct      --> (int16_t) pointer to an array for the measurements with the following order:
   *                                    A_X, A_Y, A_Z, G_X, G_Y, G_Z.
   */

  // --- Read the values ---
  readMpuMeasurements(values_funct);

  /*
  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Raw values:"));
    Serial.println(String(*(values_funct)));
    Serial.println(String(*(values_funct + 1)));
    Serial.println(String(*(values_funct + 2)));
    Serial.println(String(*(values_funct + 3)));
    Serial.println(String(*(values_funct + 4)));
    Serial.println(String(*(values_funct + 5)));
    if ((values_funct[0] == 0X7FFF) || (values_funct[0] == 0X8000)) Serial.println("Accel-X overflow!--------------------------------------------------------------");
    if ((values_funct[1] == 0X7FFF) || (values_funct[1] == 0X8000)) Serial.println("Accel-Y overflow!--------------------------------------------------------------");
    if ((values_funct[2] == 0X7FFF) || (values_funct[2] == 0X8000)) Serial.println("Accel-Z overflow!--------------------------------------------------------------");
    if ((values_funct[3] == 0X7FFF) || (values_funct[3] == 0X8000)) Serial.println("Gyro-X overflow!---------------------------------------------------------------");
    if ((values_funct[4] == 0X7FFF) || (values_funct[4] == 0X8000)) Serial.println("Gyro-Y overflow!---------------------------------------------------------------");
    if ((values_funct[5] == 0X7FFF) || (values_funct[5] == 0X8000)) Serial.println("Gyro-Z overflow!---------------------------------------------------------------");
  #endif
  */

}

void MpuDev::getRefinedValues(double *measurements_funct) {
  /* This function gets the refined accelerometer and gyroscope measurements.
   * 
   * Parameters:
   *      @param *measurements_funct      --> (double) pointer to an array for the measurements with the following order:
   *                                                   A_X, A_Y, A_Z, G_X, G_Y, G_Z.
   */

  int16_t raw_values[6];
  
  // --- Get the raw values ---
  getParameter6(raw_values);

  // --- Refine values ---
  for(uint8_t i = 0; i < 6; i++){

    // -- Offset correction --
    *(measurements_funct + i) = ((double) *(raw_values + i) - *(offset_correction + i));

    // -- Obtain magnitudes --
    if ( i < 3){  // Accelerations 
      *(measurements_funct + i) /=  accel_1g_value;
    }else{  // Gyroscopes
      *(measurements_funct + i) *= (M_PI / (180 * gyro_1dps_value));
    }
  }
  /*
  // Debug
  #ifdef DEBUG_MODE_MPU
    Serial.println(F("Raw VS refined values:"));
    Serial.println(String(*(raw_values)) + "  -->  " + String(*(measurements_funct)));
    Serial.println(String(*(raw_values + 1)) + "  -->  " + String(*(measurements_funct + 1)));
    Serial.println(String(*(raw_values + 2)) + "  -->  " + String(*(measurements_funct + 2)));
    Serial.println(String(*(raw_values + 3)) + "  -->  " + String(*(measurements_funct + 3)));
    Serial.println(String(*(raw_values + 4)) + "  -->  " + String(*(measurements_funct + 4)));
    Serial.println(String(*(raw_values + 5)) + "  -->  " + String(*(measurements_funct + 5)));
  #endif 
  */
}


//            **************************
//            *     KALMAN FILTER      *
//            **************************

void MpuDev::integrate(double d_time, double *angular_speed_1, double *angular_speed_2, double *integration_result) {
  /*
   * This function performs the trapezoidal numeric integration of the angular speed.
   *
   * Parameters:
   *      @param d_time               --> (double) increment of time between the current and the previous measurement.
   *      @param *angular_speed_1     --> (double) Pointer to the current rotated angular speed array {omega_x, omega_y}
   *      @param *angular_speed_2     --> (double) Pointer to the previous rotated angular speed array {omega_x, omega_y}
   *      @param *integration_result  --> (double) Pointer to the array with the integration result
   */

  // --- Definition ---
  double temp_funct;

  // --- Integrate ---
  temp_funct = (d_time)/2.0;
  integration_result[0] = temp_funct * (angular_speed_1[0] + angular_speed_2[0]);
  integration_result[1] = temp_funct * (angular_speed_1[1] + angular_speed_2[1]);

  // --- Done ---
  return integration_result;
}

void MpuDev::rotate(double *measurements_ref, double *rotated_values) {
  /*
   * This function rotates the current angular speed measurement from the IMU reference to the global one.
   * The rotation is done according to the current state estimation
   *
   * Parameters:
   *      @param measurements_ref    --> (double) Pointer to the refined measurements array
   *      @param *rotated_values     --> (double) Pointer to the rotated angular speed array
   */

  // --- Rotation ---
  // -- Rotate w_x --
  rotated_values[0] = measurements_ref[3]*cos(state[0]) + measurements_ref[5]*sin(state[1]);
  // -- Rotate w_y --
  rotated_values[1] = measurements_ref[3] * (sin(state[0]) * sin(state[1])) + 
                      measurements_ref[4] * (cos(state[0])) -
                      measurements_ref[5] * (sin(state[0]) * cos(state[1]));

  // --- Done ---
}

double MpuDev::square(double x) {
  /*
   * This function just calculates the square of a number: X^2 = X*X
   *
   * Parameters:
   *      @param x                  --> Number to square
   *      @return x**2              --> Squared number
   */

  return x * x;

}

void MpuDev::accelState(double *measurements_ref, double *state_pred, double *accel_cov_funct) {
  /*
   * This function calculates the state based on the measurements from the accelerometer.
   * The prediction is done as follows:
   *      1) Normalize the measurements (we expect to measure 1g in total)
   *      [1-1] Low Pass filter the accelerometer measurements (it will be done on the MPU)
   *      2) Calculate the angles based on the position of the gravity vector
   *      [3)] Calculate the covariance based on 1) (deleted)
   * 
   * Parameters:
   *      @param *measurements_ref   --> (double) Pointer to the refined measurements array
   *      @param *state_pred         --> (double) Pointer to the accelerometer state prediction array (it will be overwritten)
   *      @param *accel_cov_funct    --> (double) Pointer to the accelerometer state prediction covariance value (it will be overwritten)
   */

  // --- Definitions ---
  double normalized_values[3];  // temp array for the normalized accel measurements
  
  // --- Normalization ---
  normalized_values[2] = sqrt(square(measurements_ref[0]) + square(measurements_ref[1]) + square(measurements_ref[2]));  // temp calculation :)
  // -- check for error --
  if (normalized_values[2] == 0) {
    for (uint8_t i = 0; i < 3; i++){  // If there is an error just return 0 and high covariance
      normalized_values[i] = 0;
    }
    *accel_cov_funct = 1000;
    return;
  }

  // -- Covariance calculation --
  if (normalized_values[2] < 1.0) 
    *accel_cov_funct = 1 - normalized_values[2];
  else 
    *accel_cov_funct = normalized_values[2] - 1;

  *accel_cov_funct = normalized_values[2] + (1 + (10 * *accel_cov_funct * *accel_cov_funct));


  /*
  // -- continue --
  // covariance calculation
  if (normalized_values[2] < 1.0) {
    *accel_cov_funct = accel_covariance / normalized_values[2];
  } else {
    *accel_cov_funct = accel_covariance * normalized_values[2];
  }
  */

  // -- Normalize values --
  for (uint8_t i = 0; i < 3; i++) {
    normalized_values[i] = measurements_ref[i] / normalized_values[2];
  }

  // --- State calculation ---
  state_pred[0] = atan2(normalized_values[1], sqrt(square(normalized_values[0]) + square(normalized_values[2])));
  state_pred[1] = -atan2(normalized_values[0], sqrt(square(normalized_values[1]) + square(normalized_values[2])));
}

double* MpuDev::simplifiedKF(unsigned long current_time) {
  /*
   * This function estimates the X and Y angles (State of the system) applying a simplified version of the Kalman filter
   * To the measurements of the gyro and accel and the previous measurements. The simplification is done first assuming that the z angle is static,
   * when calculating the covariance it is assumed that the state covariance doesn't contribute significantly to the covariance when a rotation is
   * applied and that the main acceleration will be produced by gravity. All these approximations are only to reduce the amount of computation needed.
   *
   * The Kalman filter goes as follows:
   *      1) Use the gyro measurements and the previous state estimation to predict the current state and calculate the variance of this prediction.
   *      2) Use the accelerometer to calculate the state by the position of the gravity vector and the covariance of this calculation.
   *      3) Calculate the Kalman gain.
   *      4) Update the prediction in 1 with the new information and update the covariance matrix of the state
   * 
   * Parameters:
   *      @param *current_time      --> (unsigned long) millis() time when the measurements were taken (it should be obtained with the interrupt)
   *      @return *state_kalman     --> (double) Pointer to the state array (state = X_angle, Y_angle)
   */

  // --- Initialization ---
  // -- Definitions --
  double measurements_funct[6];     // Array for the refined measurements
  double angular_speed_1[2];        // Array for the current rotated speed
  double delta_time;                // Time interval since the filter was called
  double state_accel[2];            // State calculation from the accelerometer values
  double integration_result[2];     // This is just to hold the integration results
  double temp_funct[2];             // Just to hold temporal calculations
  // -- Get Measurements --
  getRefinedValues(measurements_funct);

  if (mpu_state_global != MPU_CORRECT) return state;

  // --- Prediction ---
  // -- Rotate the angular speeds --
  rotate(measurements_funct, angular_speed_1);
  // -- Integrate --
  delta_time = ((double)(current_time - prev_time)/1000.0);
  integrate(delta_time, angular_speed_1, rotated_ang_speed_prev, temp_funct);
  state[0] += temp_funct[0];
  state[1] += temp_funct[1];
  // -- Covariance --
  state_covariance[0] += square(delta_time) * gyro_covariance;
  state_covariance[1] = state_covariance[0];

  // --- Innovation ---
  // -- State calculation with the accelerometer --
  accelState(measurements_funct, state_accel, temp_funct);
  // -- obtain innovation --
  state_accel[0] -= state[0];
  state_accel[1] -= state[1];

  // --- Update ---
  // -- x-axis --
  temp_funct[0] = state_covariance[0]/(state_covariance[0] + temp_funct[0]);  // Kalman gain
  state[0] += (temp_funct[0] * state_accel[0]);
  state_covariance[0] = (1 - temp_funct[0]) * state_covariance[0];
  // -- y-axis --
  temp_funct[0] = state_covariance[0]/(state_covariance[0] + accel_covariance);  // Kalman gain
  state[1] += (temp_funct[0] * state_accel[1]);
  state_covariance[1] = (1 - temp_funct[0]) * state_covariance[1];

  // --- Done ---
  rotated_ang_speed_prev[0] = angular_speed_1[0];
  rotated_ang_speed_prev[1] = angular_speed_1[1];
  prev_time = current_time;
  return state;
}

double* MpuDev::testGyroEst(unsigned long current_time) {
  /*
   * This function is just to test the gyro estimation.
   */

  // --- Initialization ---
  // -- Definitions --
  double measurements_funct[6];     // Array for the refined measurements
  double angular_speed_1[2];        // Array for the current rotated speed
  double delta_time;                // Time interval since the filter was called
  double integration_result[2];     // This is just to hold the integration results

  // -- Get Measurements --
  getRefinedValues(measurements_funct);

  if (mpu_state_global != MPU_CORRECT) return state;

  // --- Prediction ---
  // -- Rotate the angular speeds --
  angular_speed_1[0] = measurements_funct[3]*cos(state_gyro[0]) + measurements_funct[5]*sin(state_gyro[1]);
  // -- Rotate w_y --
  angular_speed_1[1] = measurements_funct[3] * (sin(state_gyro[0]) * sin(state_gyro[1])) + 
                       measurements_funct[4] * (cos(state_gyro[0])) -
                       measurements_funct[5] * (sin(state_gyro[0]) * cos(state_gyro[1]));
  // -- Integrate --
  delta_time = ((double)(current_time - prev_time)/1000.0);
  integrate(delta_time, angular_speed_1, rotated_ang_speed_prev_2, integration_result);
  state_gyro[0] += integration_result[0];
  state_gyro[1] += integration_result[1];
  // -- Covariance --
  state_gyro_cov[0] += square(delta_time) * gyro_covariance;
  state_gyro_cov[1] = state_gyro_cov[0];

  // --- Done ---
  rotated_ang_speed_prev_2[0] = angular_speed_1[0];
  rotated_ang_speed_prev_2[1] = angular_speed_1[1];
  return state_gyro;
}

double* MpuDev::testAccelEst(unsigned long current_time) {
  /*
   * This is just to test the accelerometer estimation.
   */
  double temp_funct;            // covariance temp value
  // -- Get Measurements --

   // -- Definitions --
  double measurements_funct[6];     // Array for the refined measurements
  getRefinedValues(measurements_funct);

  if (mpu_state_global != MPU_CORRECT) return state;

  // -- State calculation with the accelerometer --
  accelState(measurements_funct, state_accel_est, &temp_funct);
  return state_accel_est;
}

void MpuDev::initializeMeasurements() {
  /*
   * This function resets the data to start with the estimations from the origin.
   *
   * Parameters:
   *      @param initial_time     --> (unsigned long) millis() time when the measurements were taken (it should be obtained with the interrupt)
   */

  mpu_data_ready = false;

  while (!mpu_data_ready) {
    // Just wait
  }

  resetSignalPath();

  mpu_data_ready = false;

  while (!mpu_data_ready) {
    // Just wait
  }

  prev_time = time_buffer;
}