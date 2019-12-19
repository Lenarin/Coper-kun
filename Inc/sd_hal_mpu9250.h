#ifndef DRIVERS_MYLIB_SD_HAL_H_
#define DRIVERS_MYLIB_SD_HAL_H_

/* Default I2C address */
#define I2C_ADDR			0xD0

/* Who I am register value */
#define I_AM				0x68

/* MPU9250 registers */
#define AUX_VDDIO			0x01
#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define MOTION_THRESH		0x1F
#define INT_PIN_CFG			0x37
#define INT_ENABLE			0x38
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define MOT_DETECT_STATUS	0x61
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A
#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2			0x6C
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL			0x73
#define FIFO_R_W			0x74
#define WHO_AM_I			0x75
#define FIFO_EN          	0x23
#define I2C_MST_CTRL    	0x24
#define XG_OFFS_USRH     	0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL    	0x14
#define YG_OFFS_USRH     	0x15
#define YG_OFFS_USRL     	0x16
#define ZG_OFFS_USRH     	0x17
#define ZG_OFFS_USRL     	0x18
#define XA_OFFSET_H     	0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   	0x07
#define YA_OFFSET_H      	0x08
#define YA_OFFSET_L_TC   	0x09
#define ZA_OFFSET_H      	0x0A
#define ZA_OFFSET_L_TC   	0x0B
#define SELF_TEST_X      	0x0D
#define SELF_TEST_Y      	0x0E
#define SELF_TEST_Z      	0x0F
#define SELF_TEST_A      	0x10

/* Gyro sensitivities in degrees/s */
#define GYRO_SENS_250		((float) 131)
#define GYRO_SENS_500		((float) 65.5)
#define GYRO_SENS_1000		((float) 32.8)
#define GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define ACCE_SENS_2			((float) 16384)
#define ACCE_SENS_4			((float) 8192)
#define ACCE_SENS_8			((float) 4096)
#define ACCE_SENS_16		((float) 2048)

// Magnetometr

#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value


/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/**
 * @brief  Data rates predefined constants
 * @{
 */
#define SD_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define SD_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define SD_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define SD_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define SD_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define SD_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define SD_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define SD_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup SD_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU9250 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	SD_Device_0 = 0x00, /*!< AD0 pin is set to low */
	SD_Device_1 = 0x02  /*!< AD0 pin is set to high */
} SD_Device;

/**
 * @brief  MPU9250 result enumeration
 */
typedef enum  {
	SD_Result_Ok = 0x00,          /*!< Everything OK */
	SD_Result_Error,              /*!< Unknown error */
	SD_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	SD_Result_DeviceInvalid       /*!< Connected device with address is not MPU9250 */
} SD_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	SD_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	SD_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	SD_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	SD_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} SD_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	SD_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	SD_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	SD_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	SD_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} SD_Gyroscope;

/**
 * @brief  Main MPU9250 structure
 */
typedef struct  {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} SD_MPU9250;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} SD_Interrupt;

typedef struct imuStruct{
	float yaw;
	float pitch;
	float roll;
	float lin_ax;
	float lin_ay;
	float lin_az;
} imuStruct;

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

imuStruct imuData;


/**
 * @}
 */

/**
 * @defgroup SD_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU9250 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref SD_t structure
 * @param  DeviceNumber: MPU9250 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be SD_Device_0,
 *          but if AD0 pin is high, then you should use SD_Device_1
 *
 *          Parameter can be a value of @ref SD_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref SD_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref SD_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - SD_Result_t: Everything OK
 *            - Other member: in other cases
 */
SD_Result SD_MPUInit(SD_Accelerometer AccelerometerSensitivity, SD_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_Gyroscope_t enumeration
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_SetGyroscope(SD_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_Accelerometer_t enumeration
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_SetAccelerometer(SD_Accelerometer AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_SetDataRate(uint8_t rate);


/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_EnableInterrupts();

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_DisableInterrupts();

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref SD_t structure indicating MPU9250 device
 * @param  *InterruptsStruct: Pointer to @ref SD_Interrupt_t structure to store status in
 * @retval Member of @ref SD_Result_t enumeration
 */
SD_Result SD_ReadInterrupts(SD_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref SD_t structure to store data to
 * @retval Member of @ref SD_Result_t:
 *            - SD_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_Result SD_ReadAccelerometer();

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref SD_t structure to store data to
 * @retval Member of @ref SD_Result_t:
 *            - SD_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_Result SD_ReadGyroscope();

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_t structure to store data to
 * @retval Member of @ref SD_Result_t:
 *            - SD_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_Result SD_ReadTemperature();

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_t structure to store data to
 * @retval Member of @ref SD_Result_t:
 *            - SD_Result_Ok: everything is OK
 *            - Other: in other cases
 */
SD_Result SD_ReadAll();

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
char readByte(uint8_t subAddress);
void readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest);
HAL_StatusTypeDef writeByte(uint8_t subAddress, uint8_t data);
void MPU9250SetConnection(I2C_HandleTypeDef* I2Cx, SD_Device DeviceNumber);
void gyroCalibration();
void accelCalibration();
imuStruct* imuGetPtr();
void magcalMPU9250();
SD_Result initAK8963();
void MPU9250_TakeAndCalcData(float dt);

float gx_out, gy_out, gz_out;


#endif /* DRIVERS_MYLIB_SD_HAL_H_ */
