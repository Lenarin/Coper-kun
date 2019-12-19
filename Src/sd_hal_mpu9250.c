#include "sd_hal_MPU9250.h"
#include "main.h"

uint8_t Ascale = AFS_2G; //AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; //GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = { 0, 0, 0 }, magBias[3] = { 0, 0, 0 }; // Factory mag calibration and mag bias
float magScale[3] = { 0, 0, 0 };
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t tempCount; // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate

// parameters for 6 DoF sensor fusion calculations
float PI = 3.14159265358979323846f;
float GyroMeasError; // = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = 0.7f;//0.5f;  // = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift; // = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = 0.05f;//0.01f; // = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.05f

// imuStruct imuData;
//float pitch, yaw, roll;
float deltat = 0.0f;             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval                               // used to calculate integration interval
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };           // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method

uint32_t MPU9250_lastUpdate, MPU9250_firstUpdate, MPU9250_now; // used to calculate integration interval
float MPU9250_deltat; // = 0.0f;                              // integration interval for both filter schemes

int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx_raw, my_raw, mz_raw;
float ax, ay, az, gx, gy, gz, mx, my, mz;
float prev_ax, prev_ay;
float exInt, eyInt, ezInt, q0_last, q1_last, q2_last;

float GyroMeasDriftX = 0, GyroMeasDriftY = 0, GyroMeasDriftZ = 0;

float zAxis[3] = {1, 1, 1}, yAxis[3] = {1, 1, 1}, xAxis[3] = {1, 1, 1};


I2C_HandleTypeDef* Handle;
uint8_t address = 0x68;

void getMres() {
	switch (Mscale) {

	case MFS_14BITS:
		mRes = 10.0 * 4219.0 / 8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0 * 4219.0 / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void getGres() {
	switch (Gscale) {

	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void getAres() {
	switch (Ascale) {

	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}

HAL_StatusTypeDef writeByte(uint8_t subAddress, uint8_t data)
{
	char d[2];
	d[0] = subAddress;
	d[1] = data;
	return HAL_I2C_Master_Transmit(Handle,(uint16_t)address, (uint8_t *)d, 2, 1000);
}

char readByte(uint8_t subAddress)
{
	char data[1];
	char data_write[1];
	data_write[0] = subAddress;
	HAL_I2C_Master_Transmit(Handle, address, (uint8_t *)data_write, 1, 1000);
	HAL_I2C_Master_Receive(Handle, address, (uint8_t *)data, 1, 1000);
	return data[0];
}

void inline readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	 char data[14];
	 char data_write[1];
	 data_write[0] = subAddress;
	 HAL_I2C_Master_Transmit(Handle, address, (uint8_t *)data_write, 1, 1000); // no stop
	 HAL_I2C_Master_Receive(Handle, address, (uint8_t *)data, count, 1000);
	 for(int ii = 0; ii < count; ii++) {
	  dest[ii] = data[ii];
	 }
}

HAL_StatusTypeDef writeMagByte(uint8_t subAddress, uint8_t data)
{
	char d[2];
	d[0] = subAddress;
	d[1] = data;
	return HAL_I2C_Master_Transmit(Handle,(uint16_t)AK8963_ADDRESS, (uint8_t *)d, 2, 1000);
}

char readMagByte(uint8_t subAddress)
{
	char data[1];
	char data_write[1];
	data_write[0] = subAddress;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)AK8963_ADDRESS, (uint8_t *)data_write, 1, 1000);
	HAL_I2C_Master_Receive(Handle, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, 1, 1000);
	return data[0];
}

void inline readMagBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	char data[14];
	char data_write[1];
	data_write[0] = subAddress;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)AK8963_ADDRESS, (uint8_t *)data_write, 1, 1000); // no stop
	HAL_I2C_Master_Receive(Handle, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, count, 1000);
	for(int ii = 0; ii < count; ii++) {
		dest[ii] = data[ii];
	}
}

void MPU9250SetConnection(I2C_HandleTypeDef* I2Cx, SD_Device DeviceNumber)
{
	Handle = I2Cx;
	address = I2C_ADDR | (uint8_t)DeviceNumber;
}

SD_Result initAK8963()
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeMagByte(AK8963_CNTL, 0x00); // Power down magnetometer
	HAL_Delay(10);
	writeMagByte(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	HAL_Delay(10);
	readMagBytes(AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
	writeMagByte(AK8963_CNTL, 0x00); // Power down magnetometer
	HAL_Delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeMagByte(AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	HAL_Delay(10);

#if DEBUG == 1
	char c = readMagByte(AK8963_WHO_AM_I);
	printf("KA8963 WHO AM I = %#x; Must be 0x48\n", c);
#endif

	magBias[0] = 370.659637;
	magBias[1] = 175.087387;
	magBias[2] = -82.175964;
	magScale[0] = 0.972464;
	magScale[1] = 0.931944;
	magScale[2] = 1.112769;

	getMres();
	return SD_Result_Ok;
}


SD_Result SD_MPUInit(SD_Accelerometer AccelerometerSensitivity, SD_Gyroscope GyroscopeSensitivity)
{
	uint8_t d[2];
	Gscale = GyroscopeSensitivity;
	Ascale = AccelerometerSensitivity;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,address,10,20)!=HAL_OK)
	{
				//return SD_Result_Error;
	}

	writeByte(PWR_MGMT_1, 0x00);
	HAL_Delay(10);
	writeByte(PWR_MGMT_1, 0x01);

#if DEBUG == 1
	char temp = readByte(WHO_AM_I);
	printf("MPU9250 WHO AM I = %#x; Must be 0x71\n", temp);
#endif

	//writeByte(CONFIG, 0x03);
	writeByte(CONFIG, 0x06);

	//writeByte(SMPLRT_DIV, 0x01);
	writeByte(SMPLRT_DIV, 0x00);

	//uint8_t c =  readByte(GYRO_CONFIG);
	//writeByte(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	//writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	//writeByte(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	uint8_t c =  readByte(GYRO_CONFIG);
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	writeByte(GYRO_CONFIG, c );


	c =  readByte(ACCEL_CONFIG);
	writeByte(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	writeByte(INT_PIN_CFG, 0x22);
	writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

	getGres();
	getAres();
	return SD_Result_Ok;
}

void resetMPU9250() {
// reset device
	writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(10);
}

void calibrateMPU9250()
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(10);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(PWR_MGMT_1, 0x01);
  writeByte(PWR_MGMT_2, 0x00);
  HAL_Delay(20);

// Configure device for bias calculation
  writeByte(INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(FIFO_EN, 0x00);      // Disable FIFO
  writeByte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(10);

// Configure MPU9250 gyro and accelerometer for bias calculation
  writeByte(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(USER_CTRL, 0x40);   // Enable FIFO
  writeByte(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  HAL_Delay(8); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(XG_OFFS_USRH, data[0]);
  writeByte(XG_OFFS_USRL, data[1]);
  writeByte(YG_OFFS_USRH, data[2]);
  writeByte(YG_OFFS_USRL, data[3]);
  writeByte(ZG_OFFS_USRH, data[4]);
  writeByte(ZG_OFFS_USRL, data[5]);

  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
//  writeByte(XA_OFFSET_H, data[0]);
//  writeByte(XA_OFFSET_L_TC, data[1]);
//  writeByte(YA_OFFSET_H, data[2]);
//  writeByte(YA_OFFSET_L_TC, data[3]);
//  writeByte(ZA_OFFSET_H, data[4]);
//  writeByte(ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;

#if DEBUG == 1
   printf("accelBias[0]: %f\n", accelBias[0]);
   printf("accelBias[1]: %f\n", accelBias[1]);
   printf("accelBias[2]: %f\n", accelBias[2]);
   printf("gyroBias[0]: %f\n", gyroBias[0]);
   printf("gyroBias[1]: %f\n", gyroBias[1]);
   printf("gyroBias[2]: %f\n", gyroBias[2]);
#endif
}

void magcalMPU9250()
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	printf("Start magnetometr calibration: \n");
	HAL_Delay(400);

	// shoot for ~fifteen seconds of mag data
	if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if(Mmode == 0x06) sample_count = 2000;  // at 100 Hz ODR, new mag data is available every 10 ms
	for(ii = 0; ii < sample_count; ii++) {
		readMagData(); // Read the mag data

		if(mx_raw > mag_max[0]) mag_max[0] = mx_raw;
		if(mx_raw < mag_min[0]) mag_min[0] = mx_raw;
		if(my_raw > mag_max[1]) mag_max[1] = my_raw;
		if(my_raw < mag_min[1]) mag_min[1] = my_raw;
		if(mz_raw > mag_max[2]) mag_max[2] = mz_raw;
		if(mz_raw < mag_min[2]) mag_min[2] = mz_raw;

		if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(Mmode == 0x06) HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}


	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	magBias[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	magBias[1] = (float) mag_bias[1]*mRes*magCalibration[1];
	magBias[2] = (float) mag_bias[2]*mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	magScale[0] = avg_rad/((float)mag_scale[0]);
	magScale[1] = avg_rad/((float)mag_scale[1]);
	magScale[2] = avg_rad/((float)mag_scale[2]);

	printf("magBias x : %f\n", magBias[0]);
	printf("magBias y : %f\n", magBias[1]);
	printf("magBias z : %f\n", magBias[2]);
	printf("magScale x : %f\n", magScale[0]);
	printf("magScale y : %f\n", magScale[1]);
	printf("magScale z : %f\n", magScale[2]);

}

void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4] = {0, 0, 0, 0};
   uint8_t selfTest[6];
   float factoryTrim[6];

   // Configure the accelerometer for self-test
   writeByte(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(25);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); 	// FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); 	// FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); 	// FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));             	// FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));            	// FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));            	// FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }

}


void inline readAccelData()
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  ax_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  ay_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  az_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void inline readGyroData()
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gx_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gy_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gz_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void inline readMagData()
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readMagByte(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readMagBytes(AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    mx_raw = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    my_raw = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
    mz_raw = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
   }
  }
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx,
		float gy, float gz, float mx, float my, float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	if(mx*mx+my*my+mz*mz == 0) MadgwickQuaternionUpdate(ax,ay,az,gx,gy,gz);
	// Normalise accelerometer measurement
	/*
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	*/
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	/*
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	*/
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
			+ _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
			+ my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
			+ _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	float _8bx = 2.0f * _4bx;
	float _8bz = 2.0f * _4bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2 * (q2q4 - q1q3) - ax) + _2q2 * (2 * (q1q2 + q3q4) - ay)
			+ -_4bz * q3
					* (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
			+ (-_4bx * q4 + _4bz * q2)
					* (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
			+ _4bx * q3
					* (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2 * (q2q4 - q1q3) - ax) + _2q1 * (2 * (q1q2 + q3q4) - ay)
			+ -4 * q2 * (2 * (0.5 - q2q2 - q3q3) - az)
			+ _4bz * q4
					* (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
			+ (_4bx * q3 + _4bz * q1)
					* (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
			+ (_4bx * q4 - _8bz * q2)
					* (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2 * (q2q4 - q1q3) - ax) + _2q4 * (2 * (q1q2 + q3q4) - ay)
			+ (-4 * q3) * (2 * (0.5 - q2q2 - q3q3) - az)
			+ (-_8bx * q3 - _4bz * q1)
					* (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
			+ (_4bx * q2 + _4bz * q4)
					* (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
			+ (_4bx * q1 - _8bz * q3)
					* (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2 * (q2q4 - q1q3) - ax) + _2q3 * (2 * (q1q2 + q3q4) - ay)
			+ (-_8bx * q4 + _4bz * q2)
					* (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
			+ (-_4bx * q1 + _4bz * q3)
					* (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
			+ (_4bx * q2)
					* (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
	/*
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
	norm = 1.0f / norm;
	*/
	norm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	gx -= 2 * (q1 * s2 - q2 * s1 - q3 * s4 + q4 * s3) * MPU9250_deltat * zeta;
	gy -= 2 * (q1 * s3 + q2 * s4 - q3 * s1 - q4 * s2) * MPU9250_deltat * zeta;
	gz -= 2 * (q1 * s4 - q2 * s3 + q3 * s2 - q4 * s1) * MPU9250_deltat * zeta;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * MPU9250_deltat;
	q2 += qDot2 * MPU9250_deltat;
	q3 += qDot3 * MPU9250_deltat;
	q4 += qDot4 * MPU9250_deltat;
	/*
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // normalise quaternion
	norm = 1.0f / norm;
	*/
	norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

	imuData.yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0f / PI;
	imuData.roll = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180.0f / PI;
	imuData.pitch = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / PI;

}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                             // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f; // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	//            float _2q1q3 = 2.0f * q1 * q3;
	//            float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	/*
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	*/
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(
			hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3
					+ hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * MPU9250_deltat * zeta;
	gbiasy += gerry * MPU9250_deltat * zeta;
	gbiasz += gerrz * MPU9250_deltat * zeta;
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 - (beta * hatDot1)) * MPU9250_deltat;
	q2 += (qDot2 - (beta * hatDot2)) * MPU9250_deltat;
	q3 += (qDot3 - (beta * hatDot3)) * MPU9250_deltat;
	q4 += (qDot4 - (beta * hatDot4)) * MPU9250_deltat;

	// Normalize the quaternion
	/*
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // normalise quaternion
	norm = 1.0f / norm;
	*/
	norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

	imuData.yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0f / PI;
	imuData.roll = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180.0f / PI;
	imuData.pitch = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / PI;

}

void gyroCalibration() {
	uint8_t col = 5000;
	float gz_sum = 0, gx_sum = 0, gy_sum = 0;

	uint8_t count = 0;
	while (count < col) {
		if (readByte(INT_STATUS) & 0x01) {
			readGyroData();
			gx = (float) gx_raw * gRes;
			gy = (float) gy_raw * gRes;
			gz = (float) gz_raw * gRes;

			gx_sum += gx;
			gy_sum += gy;
			gz_sum += gz;

			count++;
		}
	}

	gyroBias[0] = (float)gx_sum / (float)col;
	gyroBias[1] = (float)gy_sum / (float)col;
	gyroBias[2] = (float)gz_sum / (float)col;

#if DEBUG == 1
	printf("GyroMeasDriftX: %f\n", gyroBias[0]);
	printf("GyroMeasDriftY: %f\n", gyroBias[1]);
	printf("GyroMeasDriftZ: %f\n", gyroBias[2]);
#endif

}

void accelCalibration() {
	uint8_t col = 5000;
	float az_sum = 0, ax_sum = 0, ay_sum = 0;

	uint8_t count = 0;
	while (count < col) {
		if (readByte(INT_STATUS) & 0x01) {
			readAccelData();
			ax = (float) ax_raw * aRes;
			ay = (float) ay_raw * aRes;
			az = (float) az_raw * aRes;

			//ax = xAxis[0] * ax + yAxis[0] * ay + zAxis[0] * az;
			//ay = xAxis[1] * ax + yAxis[1] * ay + zAxis[1] * az;
			//az = xAxis[2] * ax + yAxis[2] * ay + zAxis[2] * az;

			ax_sum += ax;
			ay_sum += ay;
			az_sum += az;

			count++;
		}
	}

	accelBias[0] = (float)ax_sum / (float)col;
	accelBias[1] = (float)ay_sum / (float)col;
	accelBias[2] = (float)az_sum / (float)col + 1;

#if DEBUG == 1
	printf("AccelMeasDriftX: %f\n", accelBias[0]);
	printf("AccelMeasDriftY: %f\n", accelBias[1]);
	printf("AccelMeasDriftZ: %f\n", accelBias[2]);
#endif
}

void accelCalibrationEx() {
	readAccelData();
	// Calc -g
	ax = -(float) ax_raw * aRes;
	ay = -(float) ay_raw * aRes;
	az = -(float) az_raw * aRes;

	float gnorm = sqrt(ax * ax + ay * ay + az * az);

	zAxis[0] = ax / gnorm;
	zAxis[1] = ay / gnorm;
	zAxis[2] = az / gnorm;

	ax = 0;
	ay = zAxis[2];
	az = -zAxis[1];
	gnorm = sqrt(ax * ax + ay * ay + az * az);

	yAxis[0] = ax / gnorm;
	yAxis[1] = ay / gnorm;
	yAxis[2] = az / gnorm;


	// xAsix = cross(yAxis, zAxis)
	xAxis[0] = zAxis[1] * yAxis[2] - zAxis[2] * yAxis[1];
	xAxis[1] = zAxis[2] * yAxis[0] - zAxis[0] * yAxis[2];
	xAxis[2] = zAxis[0] * yAxis[1] - zAxis[1] * yAxis[0];
}

void MPU9250_TakeAndCalcData(float dt) {
	readAccelData();
	readGyroData();
	readMagData();

	ax = (float) ax_raw * aRes - accelBias[0];
	ay = (float) ay_raw * aRes - accelBias[0];
	az = (float) az_raw * aRes - accelBias[0];

	//ax = xAxis[0] * ax + yAxis[0] * ay + zAxis[0] * az - accelBias[0];
	//ay = xAxis[1] * ax + yAxis[1] * ay + zAxis[1] * az - accelBias[1];
	//az = xAxis[2] * ax + yAxis[2] * ay + zAxis[2] * az - accelBias[2];

	gx = (float) gx_raw * gRes - gyroBias[0];
	gy = (float) gy_raw * gRes - gyroBias[1];
	gz = (float) gz_raw * gRes - gyroBias[2];

	gx_out = gx;
	gy_out = gy;
	gz_out = gz;

	mx = (float)mx_raw * mRes * magCalibration[0] - magBias[0] ;
	my = (float)my_raw * mRes * magCalibration[1] - magBias[1] ;
	mz = (float)mz_raw * mRes * magCalibration[2] - magBias[2] ;
	//printf("%f %f %f\n",mx,my,mz);



	//MPU9250_now = HAL_GetTick();
	//MPU9250_deltat = (float) ((MPU9250_now - MPU9250_lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
	//MPU9250_lastUpdate = MPU9250_now;


	MPU9250_deltat = dt;

}

void AHRS_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bz, by;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float q0_last, q1_last, q2_last;

    //auxiliary variables to reduce number of repeated operations
    float q0q0 = q[0]*q[0];
    float q0q1 = q[0]*q[1];
    float q0q2 = q[0]*q[2];
    float q0q3 = q[0]*q[3];
    float q1q1 = q[1]*q[1];
    float q1q2 = q[1]*q[2];
    float q1q3 = q[1]*q[3];
    float q2q2 = q[2]*q[2];
    float q2q3 = q[2]*q[3];
    float q3q3 = q[3]*q[3];

    //normalise the measurements
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    norm = invSqrt(mx*mx + my*my + mz*mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    //compute reference direction of flux
    hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

    // bx = sqrtf((hx*hx) + (hy*hy));
    by = sqrtf((hx*hx) + (hy*hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        // integral error scaled integral gain
        exInt = exInt + ex*Ki * MPU9250_deltat;
        eyInt = eyInt + ey*Ki * MPU9250_deltat;
        ezInt = ezInt + ez*Ki * MPU9250_deltat;

        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }

    // save quaternion
    q0_last = q[0];
    q1_last = q[1];
    q2_last = q[2];

    // integrate quaternion rate and normalise (Picard first order)
    q[0] = q0_last + (-q1_last*gx - q2_last*gy - q[3]*gz) * MPU9250_deltat;
    q[1] = q1_last + ( q0_last*gx + q2_last*gz - q[3]*gy) * MPU9250_deltat;
    q[2] = q2_last + ( q0_last*gy - q1_last*gz + q[3]*gx) * MPU9250_deltat;
    q[3] = q[3] + ( q0_last*gz + q1_last*gy - q2_last*gx) * MPU9250_deltat;

    // normalise quaternion
    norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] = q[0] * norm;    //w
    q[1] = q[1] * norm;    //x
    q[2] = q[2] * norm;    //y
    q[3] = q[3] * norm;    //z

    // Quaternion to euler angle
    imuData.roll  =  -asin(2*q[0]*q[2] - 2*q[1]*q[3]) * 180.0f / PI;
    imuData.pitch =  atan2(2*q[0]*q[1] + 2*q[2]*q[3], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]) * 180.0f / PI;
    imuData.yaw   =  atan2(2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[2]*q[2] - 2*q[3]*q[3]) * 180.0f / PI;

}

void MPU9250_CalcYPR() {
	MPU9250_MadgwickQuaternionUpdate(ax, -ay, -az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, -my, mx, -mz);
	//MadgwickQuaternionUpdate(ax, -ay, -az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f);
	//AHRS_Update(gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, -ax, ay, az, -mx, my, mz);

}

imuStruct* imuGetPtr() {
	return &imuData;
}


