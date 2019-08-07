/*
                                  ********************
                                  *    MPU Library   *
                                  ********************

   ::::::::::::::::::::
   ::::Headder File::::
   ::::::::::::::::::::
   Written by: David Arnaiz
   Github: TBD

   Released under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
   license (https://creativecommons.org/licenses/by-sa/4.0/)

   - Acknolegements -
   This file uses the I2Cdev and MPU6050 libraries, which can be found in https://github.com/jrowberg/i2cdevlib. 
   Also the rest of the code is based on the examples given. Refer ot the github repository for more information about the sources.

   - MPU Library -
  This library will define the high level functions for the MPU6050 gyroscope and Accelerometer so the rest of the code is transparent to it.

  - Change Log -
  V0.0 27/12/2018 First issue
*/


//            *********************
//            *     INCLUDES      *
//            *********************
#ifndef _MPU_LIBRARY_H_
#define _MPU_LIBRARY_H_
//#include "mpu_6050_library.h"

#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include <math.h>
//#include "MPU6050.h"
//#include <I2Cdev.h>
//#include <MPU6050.h>

//volatile bool mpu_data_ready;

/*            *********************
 *            *       STATE       *
 *            *********************
 * The state variable is used to detect if there has been an error so th system may react to it.
 * The values are:
 *    0     --> Not set.
 *    1     --> MPU configured and working correctly
 *    2     --> I2C communication error
 */

// extern uint8_t mpu_state_global = 0;

// --- States of the MPU ---
#define MPU_NOT_INITIALIZED               0                 // MPU still not configured/tester/calibrated
#define MPU_CORRECT                       1                 // MPU working correctly
#define MPU_I2C_ERROR                     2                 // I2C error
#define MPU_SELF_TEST_FAILED_BASE         3                 // The MPU has failed one of the self-tests
                                                            // 3 = A_X, 4 = A_Y, 5 = A_Z, 6 = G_X, 7 = G_Y, 8 = G_Z
#define MPU_NOT_CALIBRATED                9                 // MPU needs to be calibrated
#define MPU_CALIBRATION_ERROR             10                // MPU couldn't be calibrated


/*            *********************
 *            *    PARAMETERS     *
 *            *********************
 * The paramters were are used to chanjge the basic configurations easilly
 */

//--------------------------------------------------
// Debug mode
//--------------------------------------------------
//#define DEBUG_MODE_MPU                                      // Uncomment this to use the serial debug
#ifdef DEBUG_MODE_MPU
  #define SERIAL_INIT_MPU                                   // Comment if the serial interface is initialized in the main code
  #define SERIAL_SPEED                    115200            // Serial baud
#endif

//--------------------------------------------------
// I2C BUS
//--------------------------------------------------

// --- I2C Address ---
//#define I2C_ADDRESS_HIGH                                  // Uncomment if AD0 is set high
#define I2C_ADDRESS_LOW                                     // Uncomment if AD0 is set low (this will be considered if both are commented)

// --- I2C configuration ---
#define I2C_CONFIGURE_MPU                                   // Comment if the I2C is defined in the main code
#ifdef I2C_CONFIGURE_MPU
  #define I2C_CLK_SPEED                   400000            // I2C CLK speed (set as 400k fast mode I2C)
#endif
#define I2C_TIMEOUT_CON                   100               // I2C timeout in ms
#define I2C_MPU_RETRIES                   5                 // Number of retries after a timeout

// --- Device ID ---
#define MPU_DEVICE_ID_REG                 0x75              // Register address with the device ID
#define MPU_DEVICE_ID_VALUE               0x68              // ID of the device (bits 7 and 0 are assumed to be 0)


//--------------------------------------------------
// Self-test and full-scale
//--------------------------------------------------

// --- Accelerometer ---
#define MPU_ACCELEROMETER_CONF_ADDR       0x1C              // Register address to configure the accelerometer
#define MPU_SELF_TEST_ACCEL_REG_VALUE     0xF0              // Register value to set the self-test on the accelerometer (+-8g and self-test bits set to '1')
#define MPU_DEFAULT_ACCEL_REG_VALUE       0x00              // Register value to set the accelerometer with the default configuration (+-2g) [for calibration]
#define MPU_ACCEL_CONFIG_MASK_VALUE       0xF8              // Mask for the accelerometer configuration register  
// #define MPU_ACCEL_FS_2G                                      // Uncomment this to set the working full-scale range of the accelerometer to 2g
// #define MPU_ACCEL_FS_4G                                      // Uncomment this to set the working full-scale range of the accelerometer to 4g
#define MPU_ACCEL_FS_8G                                     // Uncomment this to set the working full-scale range of the acceletometer to 8g
// #define MPU_ACCEL_FS_16G                                     // Uncommnet this to set the working full-scale range of the accelerometer to 16g
#ifdef MPU_ACCEL_FS_16G
  #define MPU_ACCEL_CONFIG_VALUE          0x18              // This will set the full-scale to 16g
  const double accel_1g_value = 2048;                       // Sensitivity at 16g full-scale
#elif defined MPU_ACCEL_FS_8G
  #define MPU_ACCEL_CONFIG_VALUE          0x10              // This will set the full-scale to 8g
  const double accel_1g_value = 4096;                       // Sensitivity at 8g full-scale 
#elif defined MPU_ACCEL_FS_4G
  #define MPU_ACCEL_CONFIG_VALUE          0x08              // This will set the full-scale to 4g
  const double accel_1g_value = 8192;                       // Sensitivity at 4g full-scale 
#else
  #define MPU_ACCEL_CONFIG_VALUE          0x00              // This will set the full-scale to 2g
  const double accel_1g_value = 16384;                      // Sensitivity at 2g full-scale. This is de default value
#endif    

// --- Gyroscope ---
#define MPU_GYRO_CONF_ADDR                0x1B              // Register address to configure the gyroscope
#define MPU_SELF_TEST_GYRO_REG_VALUE      0xE0              // Register value to set the self-test on the gyroscope (+-250dps and self-test bits set to '0')
#define MPU_DEFAULT_GYRO_REG_VALUE        0x00              // Register value to set the gyroscope with the default configuration (+-250dps) [defined below]
#define MPU_GYRO_CONFIG_MASK_VALUE        0xF8              // Mask for the gyroscope configuration register
// #define MPU_GYRO_FS_250DPS                                 // Uncomment to set the working full-scale range of the gyroscope to 250dps
// #define MPU_GYRO_FS_500DPS                                 // Uncomment to set the working full-scale range of the gyroscope to 500dsp
#define MPU_GYRO_FS_1000DPS                                // Uncomment to set the working full-scale range of the gyroscope to 1000dps
// #define MPU_GYRO_FS_2000DPS                                // Uncomment to set the working full-scale range of the gyroscope to 2000dps
#ifdef MPU_GYRO_FS_2000DPS
  #define MPU_GYRO_CONFIG_VALUE           0x18              // This will set the full-scale to 2000dps
  const double gyro_1dps_value = 16.4;                      // Sensitivity at 2000dps full-scale
#elif defined MPU_GYRO_FS_1000DPS
  #define MPU_GYRO_CONFIG_VALUE           0x10              // This will set the full-scale to 1000dps
  const double gyro_1dps_value = 32.8;                      // Sensitivity at 1000dps full-scale 
#elif defined MPU_GYRO_FS_500DPS
  #define MPU_GYRO_CONFIG_VALUE           0x08              // This will set the full-scale to 500dps
  const double gyro_1dps_value = 65.5;                      // Sensitivity at 500dps full-scale 
#else
  #define MPU_GYRO_CONFIG_VALUE           0x00              // This will set the full-scale to 250dps
  const double gyro_1dps_value = 131;                       // Sensitivity at 250dps full-scale- This is the default value
#endif

// --- Configuration ---
#define MPU_SELF_TEST_WAIT_TIME           250               // Time in ms that the program will wait while the self-test are performed
#define MPU_SELF_TEST_THRESHOLD           14                // Maximum percentage allowwed for the self-test, 14% according to the datasheet
#define MPU_SELF_TEST_RESULT_ADDR_BASE    0x0D              // Base address for the Self-test results


//--------------------------------------------------
// Configuration
//--------------------------------------------------

// --- Clock reference ---
#define MPU_CLOCK_REF_ADDR                0x1B              // Address of the register to configure the clock reference of the MPU
#define MPU_CLOCK_REF_MASK                0x07              // Mask for the clock reference configuration
#define MPU_CLOCK_ZGYRO                   0x03              // Sets the Gyro-Z PLL as reference (default used)
#define MPU_CLOCK_XGYRO                   0x01              // Sets the Gyro_x PLL as reference

// --- Digital low-pass filter ---
#define MPU_DLPF_ADDR                     0x1A              // Address of the register to configure the digital low-pass filter 
                                                            // (and disables the External Frame Synchronization)
#define MPU_DLPF_MASK                     0x3F              // Mask for the DLPF configuration
#define MPU_DLPF_REG_VALUE_DEFAULT        0x00              // Default value for the digital low-pass filter configuration. Set to 260 Hz bandwidth
                                                            // this is to get the full 8kHz oscillation frequency since the Gyro-Z PLL is being used
                                                            // if not it will be 1kHz (which is the max sample rate for the accelerometer anyway)
#define MPU_DLPF_REG_VALUE_WORKING        0x02              // Value of the DLPF register for normal operation

// --- Interrupt ---
#define MPU_INTERRUPT_CONF_ADDR           0x38              // Address to the interrupt configuration register
#define MPU_INTERRUPT_CONF_MASK           0x19              // Mask for the interrupt configuration register
#define MPU_INTERRUPT_DEFAULT             0x01              // Default value for the Interrupt configuration register. This will set an interrupt when all
                                                            // the measurements have been updated in the corresponding registers

// --- Sample rate ---
#define MPU_SAMPLE_RATE_ADDR              0x19              // Address of the register to configure the sample rate for the MPU
#define MPU_SAMPLE_RATE_DEFAULT           0x07              // Register value to set the sample rate, it will be:
                                                            //        Sample Rate = (PLL freq)/(MPU_SAMPLE_RATE_DEFAULT + 1)
                                                            // Default value sets it to 1kHz
#define MPU_SAMPLE_RATE_WORKING           0x1F              // Value of the sample rate register during normal operation. Set to 31.25Hz

// --- Signal path reset ---
#define MPU_RESET_SIGNAL_PATH_ADDR        0x68              // Address for the signal path reset register
#define MPU_RESET_SIGNAL_PATH_MASK        0x07              // Mask for the signal path reset register
#define MPU_RESET_SIGNAL_PATH_RESET       0xFF              // Value for to reset the signal path (it is assuming that the mask is correct -.-)
#define MPU_RESET_SIGNAL_PATH_DELAY       10                // Time in ms that the program will wait for the signal to be reset (It could be changed to recieving an interrupt)

// --- Low power mode ---
#define MPU_LOW_POWER_MODE_ADDR           0x6B              // Address for the power management register 1 in the MPU_GYRO_CONF_ADDR
#define MPU_LOW_POWER_MODE_MASK           0xE8              // Mask for the low power mode configuration
#define MPU_LOW_POWER_MODE_ENABLE         0x40              // Register value to set the MPU into sleep mode
#define MPU_LOW_POWER_MODE_DISABLE        0x00              // Register value to disable the sleep mode and wake the MPU


//--------------------------------------------------
// Calibration
//--------------------------------------------------

// --- Number of averages ---
#define CALIBRATION_AVERAGES              10000             // The higher the better but it will be slower (int16_t value)
#define CALIBRATION_INITIAL_AVERAGES      1000              // This value is to accelerate the offset location (int16_t value)
#define CALIBRATION_MAX_ITERATIONS        100               // This is the limit of iterations to calibrate the MPU
#define CALIBRATION_INITIAL_ERROR         5                 // Maximum error for which the number of calibrations should be increased to CALIBRATION_AVERAGES
#define CALIBRATION_MIN_ERROR             1                 // Maximum difference allowed between the high and low offset values to stop the calibration 
                                                            // process

// --- Offset correction ---
#define CALIBRATION_CORRECTION_AVERAGES   1000              // Number of averages that will be made to calculate the offset correction array
#define FAST_CALIBRATION_CORRECTION                         // Sets the calibration correction at 1kHz sample frequency if not commented

// --- Calibration adjustments ---
#define CALIBRATION_OFFSET_ADJUSTMENT     1000              // Change done to the offset when finding the initial range

// --- Calibration targets ---
#define X_ACCEL_TARGET                    0                 // Target for the X accelerometer measurement (0 by default)
#define Y_ACCEL_TARGET                    0                 // Target for the Y accelerometer measurement (0 by default)
#define Z_ACCEL_TARGET                    16384             // Target for the Z accelerometer measurement (1G by default, 
                                                            // which is max value/2 as the scale is set to 2g)
#define X_GYRO_TARGET                     0                 // Target for the X gyroscope measurement (0 by default)
#define Y_GYRO_TARGET                     0                 // Target for the Y gyroscope measurement (0 by default)
#define Z_GYRO_TARGET                     0                 // Target for the Z gyroscope measurement (0 by default)

// --- Temperature calibration threshold ---
#define CALIBRATION_MAX_TEMP_DIFF         25                // Maximum temeperature difference between the calibration temperature and the current one
                                                            // It should be considered that the temperature stabilization won't be achieved when the
                                                            // reading is done (temperature in celsius)

// --- Offsets registers ---
#define MPU_ACCEL_OFFSETS_BASE_ADDR       0x06              // Base register address for the accelerometer offset adjustment
#define MPU_GYRO_OFFSETS_BASE_ADDDR       0x13              // Base register address for the gyroscope offset adjustment          
//--------------------------------------------------
// DATA
//--------------------------------------------------

// --- Temperature ---
#define MPU_TEMP_REG_BASE                 0X41              // Base address of the registes with the temperature values 

// --- Accelerometer ---
#define MPU_ACCEL_REG_BASE                0x3B              // Base address of the registers with the accelerometer values

// --- Gyroscope ---
#define MPU_GYRO_REG_BASE                 0x43              // Base address of the registers with the gyroscope values 

//--------------------------------------------------
// EEPROM
//--------------------------------------------------
// It is defined here for the moment, maybe it will be moved...
#define MPU_EEPROM_OFFSET_ADDRESS         15                // Offset for the MPU calibration EEPROM addess. it has a size of 17 bytes


//            *********************
//            *    Definitions    *
//            *********************

//--------------------------------------------------
// I2C Bus
//--------------------------------------------------

// --- I2C Address ---

//By default the AD0 address has to be connected to gnd
#ifdef I2C_ADDRESS_HIGH
  #define I2C_ADDRESS_MPU                 B1101001          // I2C address with AD0 set to '1'
#else
  #define I2C_ADDRESS_MPU                 B1101000          // I2C address with AD0 set to '0'
#endif

//--------------------------------------------------
// Kalman Filter
//--------------------------------------------------

// --- Constants ---
const double gyro_covariance = 0.203263527368261;           // Gyroscope covariance (deg/s)^2
const double accel_covariance = 1;                        // Accelerometer covariance (m/s^2)^2



//            *********************
//            *   Class object    *
//            *********************
class MpuDev{

  public:
    // --- Variables ---
    // -- MPU state --
    uint8_t mpu_state_global;                               // State of the MPU
    // -- Kalman Filter --
    double state[2] = {0, 0};                               // State for the Kalman filter (angle X, angle Y) in rad
    double state_covariance[2] = {0, 0};                    // Covariance "matrix of the state" it is assumed to be diagonal (it shouldn't be) in rad^2
    double rotated_ang_speed_prev[2] = {0, 0};              // Previous rotated angular speed in rad/s
    unsigned long prev_time = 0;                            // Previous time stamp in ms
    // -- Testing --
    double state_gyro[2] = {0, 0};                          // State to test gyro
    double state_gyro_cov[2] = {0, 0};                      // State covariance to test gyro
    double rotated_ang_speed_prev_2[2] = {0, 0};            // Previous rotated angular speed in rad/s
    double state_accel_est[2] = {0, 0};                     // State to test accel
    double state_accel_cov[2] = {0, 0};                     // State covariance to test accel

    // --- Functions ---
    // -- Tools/other --
    bool readMpuRegisters(uint8_t address_funct, 
                          uint8_t *buffer_funct, 
                          uint8_t lenght_funct);            // This function read the value of multiple sequential registers

    bool readMpuRegister(uint8_t address_funct, 
                         uint8_t *buffer_funct);            // This function reads one register of the MPU

    bool readMpuData(uint8_t address_funct, 
                     int16_t *data_funct);                  // This function reads a int16_t value from the MPU

    bool readMpuMeasurements(int16_t *data_funct);          // This function reads a int16_t value from the MPU

    bool writeMpuRegister(uint8_t address_funct,
                          uint8_t buffer_funct, 
                          bool check_funct = true);         // This function writes one register of the MPU

    bool updateMpuRegister(uint8_t address_funct,         
                           uint8_t values_funct,
                           uint8_t mask_funct, 
                           bool check_funct = true);        // This function updates the specified bits on the register

    // -- configuration --
    bool initialize_1();                                    // First initialization, configures the device to start measuring
    bool initialize_2();                                    // Second initialization, perform the calibration and offset correction calculations
    bool checkMpu();                                        // Checks that the MPU is not damaged
    void configureMpu();                                    // Configures the MPU for normal operation
    void resetSignalPath();                                 // Resets the signal path and waits for it to be completed
    void changeFullScale(uint8_t accel_reg,                 // Sets the configuration registers for the accelerometer and gyroscope
                         uint8_t gyro_reg);             
    void setSampleRate();                                   // Sets the sample rate for the MPU
    void setLowPowerMode(bool sleep_enabled);               // Sets the MPU to low power mode or wakes it up

    // -- Calibration --
    void setOffsets(int16_t *offsets);                      // Sets the offsets of the MPU
    void calculateAverages(int16_t *averages_funct,     
                           uint16_t number_of_iterations, 
                           int16_t *targets_funct);         // Calculates the averages with the given offset values
    bool calibrate(int16_t *offsets);                       // Main function for the calibration process
    void getOffsetCorrection();                             // Gets the offset correction values
    bool checkCalibration();                                // Check if the calibration is needed or not
    bool performCalibration();                              // Does the calibration

    // EEPROM
    bool loadFromEEPROM(float *temperature_mpu,             // Load the calibration data from the EEPROM 
                        int16_t *offsets_funct);      
    void saveOnEEPROM(float *temperature_mpu,               // Store the calibration data on the EEPROM
                      int16_t *offsets_funct);        

    // Measurements
    float getTemperature();                                 // Get the temperature measurements of the MPU
    void getParameter6(int16_t *values_funct);              // Get the measurements from the MPU
    void getRefinedValues(double *measurements_funct);      // Get the refined measurements from the MPU

    // Kalman filter
    void integrate(double d_time, 
                   double *angular_speed_1, 
                   double *angular_speed_2,
                   double *integration_result);             // Trapezoidal numeric integration
    void rotate(double *measurements_ref, 
                double *rotated_values);                    // Rotate the angular speed measurements
    double square(double x);                                // Returns the squared number X^2 = X * X = X**2
    void accelState(double *measurements_ref, 
                    double *state_pred,
                    double *accel_cov_funct);               // State calculation from accelerometer measuremetns
    double* simplifiedKF(unsigned long current_time);       // Kalman Filter main function

    // Test
    double* testGyroEst(unsigned long current_time);        // Test gyro estimation
    double* testAccelEst(unsigned long current_time);       // Test accel estimation

    // TBD
    void initializeMeasurements();                          // This function is to initialize the measurements for the kalman filter
      
  private:
    int16_t offset_correction[6];                           // Offset correction array
};

#endif
