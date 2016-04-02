/* REFERENCES:
references http://www.starlino.com/imu_guide.html
reference gpio: https://software.intel.com/en-us/articles/blinking-leds-manipulating-digital-gpios-on-the-intel-galileo-board-with-the-iot
reference tcp-ip: http://www.thegeekstuff.com/2011/12/c-socket-programming/
reference interrupts, kalman and mpu6050 : links from assignment question sheet
acceleration reference: http://www.freescale.com/files/sensors/doc/app_note/AN3397.pdf 
*/

#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <tgmath.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <poll.h>
#include <pthread.h>
#include <math.h>
#include <linux/i2c-dev.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h> 

#define sampling_rate_const 3

#define IP_AD_UN  "192.168.0.69"

#define MPU6050_CLOCK_PLL_XGYRO 0x01
#define MPU6050_PWR1_CLKSEL_BIT 2
#define MPU6050_PWR1_CLKSEL_LENGTH 3
#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GCONFIG_FS_SEL_BIT 4
#define MPU6050_GCONFIG_FS_SEL_LENGTH 2
#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACONFIG_AFS_SEL_BIT 4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH 2
#define MPU6050_WHO_AM_I_BIT 6
#define MPU6050_WHO_AM_I_LENGTH 6
#define RAD_TO_DEG 180/3.14
#define DEG_TO_RAD 0.01745
#define MPU6050_PWR1_SLEEP_BIT 6
#define MPU6050_DEFAULT_ADDRESS 0x68
#define MPU6050_INTCFG_INT_LEVEL_BIT 7
#define MPU6050_INTCFG_LATCH_INT_EN_BIT 5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT 4
#define MPU6050_INTERRUPT_DATA_RDY_BIT 0
#define MPU6050_ADDRESS 0b11010010 // Address with end write bit
#define MPU6050_RA_XG_OFFS_TC 0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC 0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC 0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN 0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN 0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN 0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
#define MPU6050_RA_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL 0x18
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FF_THR 0x1D
#define MPU6050_RA_FF_DUR 0x1E
#define MPU6050_RA_MOT_THR 0x1F
#define MPU6050_RA_MOT_DUR 0x20
#define MPU6050_RA_ZRMOT_THR 0x21
#define MPU6050_RA_ZRMOT_DUR 0x22
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_I2C_SLV0_ADDR 0x25
#define MPU6050_RA_I2C_SLV0_REG 0x26
#define MPU6050_RA_I2C_SLV0_CTRL 0x27
#define MPU6050_RA_I2C_SLV1_ADDR 0x28
#define MPU6050_RA_I2C_SLV1_REG 0x29
#define MPU6050_RA_I2C_SLV1_CTRL 0x2A
#define MPU6050_RA_I2C_SLV2_ADDR 0x2B
#define MPU6050_RA_I2C_SLV2_REG 0x2C
#define MPU6050_RA_I2C_SLV2_CTRL 0x2D
#define MPU6050_RA_I2C_SLV3_ADDR 0x2E
#define MPU6050_RA_I2C_SLV3_REG 0x2F
#define MPU6050_RA_I2C_SLV3_CTRL 0x30
#define MPU6050_RA_I2C_SLV4_ADDR 0x31
#define MPU6050_RA_I2C_SLV4_REG 0x32
#define MPU6050_RA_I2C_SLV4_DO 0x33
#define MPU6050_RA_I2C_SLV4_CTRL 0x34
#define MPU6050_RA_I2C_SLV4_DI 0x35
#define MPU6050_RA_I2C_MST_STATUS 0x36
#define MPU6050_RA_INT_PIN_CFG 0x37
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_DMP_INT_STATUS 0x39
#define MPU6050_RA_INT_STATUS 0x3A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS 0x61
#define MPU6050_RA_I2C_SLV0_DO 0x63
#define MPU6050_RA_I2C_SLV1_DO 0x64
#define MPU6050_RA_I2C_SLV2_DO 0x65
#define MPU6050_RA_I2C_SLV3_DO 0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_BANK_SEL 0x6D
#define MPU6050_RA_MEM_START_ADDR 0x6E
#define MPU6050_RA_MEM_R_W 0x6F
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_WHO_AM_I 0x75

#define MAX_BUF 64
#define SYSFS_GPIO_DIR "/sys/class/gpio"


uint8_t devAddr = 0x68;
char buffer[14];


long timer;
char tcp_data[256];
pthread_mutex_t mutexlock;
int terminateflag;
// kalman variables

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double time1, time2;

double Q_anglex = 0.001; // Process noise variance for the accelerometer
double Q_biasx = 0.003; // Process noise variance for the gyro bias
double R_measurex = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise

double anglex; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double biasx; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double ratex; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double Px[2][2]; // Error covariance matrix - This is a 2x2 matrix
double Kx[2]; // Kalman gain - This is a 2x1 vector
double yx; // Angle difference
double Sx; // Estimate error

double Q_angley = 0.001; // Process noise variance for the accelerometer
double Q_biasy = 0.003; // Process noise variance for the gyro bias
double R_measurey = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise

double angley; // The angle calculated by the Kalman filter - part of the 2x1 state vector
double biasy; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
double ratey; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

int16_t gyrox_offset, gyroy_offset, gyroz_offset;

double Py[2][2]; // Error covariance matrix - This is a 2x2 matrix
double Ky[2]; // Kalman gain - This is a 2x1 vector
double yy; // Angle difference
double Sy; // Estimate error

double accX, accY, accZ;
int16_t accx_offset, accy_offset, accz_offset;
double gyroX, gyroY, gyroZ;
uint8_t i2cData[14]; // Buffer for I2C data
double roll, pitch;

int writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, char *data, uint16_t timeout);

bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readBytes(devAddr, regAddr,1 , &b,0);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeBytes(devAddr, regAddr, 1, &b);
}

unsigned long get_time()
{
  unsigned long lo, hi;
    asm( "rdtsc" : "=a" (lo), "=d" (hi) ); 
    return( lo | ((uint64_t)hi << 32) );
}

/****************************************************************
 * gpio_export
 ****************************************************************/
int gpio_export(unsigned int gpio)
{
  int fd, len;
  char buf[MAX_BUF];
 
  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    printf("gpio/export");
    close(fd);
    return fd;
  }
 
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
 
  return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int gpio_unexport(unsigned int gpio)
{
  int fd, len;
  char buf[MAX_BUF];
 
  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0) {
    printf("gpio/export");
    close(fd);
    return fd;
  }
 
  len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);
  return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int gpio_set_value(unsigned int gpio, unsigned int value)
{
  int fd, len;
  char buf[MAX_BUF];
 
  len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    printf("gpio/set-value");
    close(fd);
    return fd;
  }
 
  if (value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);
 
  close(fd);
  return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
  int fd, len;
  char buf[MAX_BUF];

  len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
  fd = open(buf, O_RDONLY | O_NONBLOCK );
  if (fd < 0) {
    printf("gpio/fd_open");
  }
  return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
  return close(fd);
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
  int fd, len;
  char buf[MAX_BUF];
 
  len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    printf("gpio/direction");
    close(fd);
    return fd;
  }
 
  if (out_flag)
    write(fd, "out", 4);
  else
    write(fd, "in", 3);
 
  close(fd);
  return 0;
}

int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, char *data, uint16_t timeout) 
{
  int8_t count = 0;
  int fd = open("/dev/i2c-0", O_RDWR);

  if (fd < 0) 
  {
    return(-1);
  }

  if (ioctl(fd, I2C_SLAVE, devAddr) < 0) 
  {
    close(fd);
    return(-1);
  }


  if (write(fd, &regAddr, 1) != 1) 
  {
    printf("\nError writing readbytes... please try again for it to work\n");
    close(fd);
    return(-1);
  }
  count = read(fd, data, length);

  if (count < 0) 
  {
    printf("\nError read... please try again for it to work\n");
    close(fd);
    return(-1);
  } 

  else if (count != length) 
  {
    close(fd);
    return(-1);
  }

  close(fd);

  return count;
}

int writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) 
{ 
  int8_t count = 0;
  uint8_t buf[128];
  int fd;

  if (length > 127) 
  {
    return(0);
  }

  fd = open("/dev/i2c-0", O_RDWR);
  
  if (fd < 0) 
  {
    return(0);
  }

  if (ioctl(fd, I2C_SLAVE, devAddr) < 0) 
  {
    close(fd);
    return(0);
  }

  buf[0] = regAddr;
  memcpy(buf+1,data,length);
  count = write(fd, buf, length+1);

  if (count < 0) 
  {
    printf("\nError writing writebytes... please try again for it to work\n");
    close(fd);
    return(0);
  } 

  else if (count != length+1) 
  {
    close(fd);
    return(0);
  }

  close(fd);
  return 1;
}

int writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {

  uint8_t b;

  if (readBytes(devAddr, regAddr, 1, &b, 0) != 0) 
  {

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return writeBytes(devAddr, regAddr, 1, &b);

  } 
  else 
  {
    return 0;
  }
}

void setInterruptMode(bool mode) {
   writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}

/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
void setInterruptLatch(bool latch) {
    writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}

/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
void setInterruptLatchClear(bool clear) {
    writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6050_RA_INT_CFG
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
void setIntDataReadyEnabled(bool enabled) {
    writeBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}

void getIntStatus() {
    readBytes(devAddr, MPU6050_RA_INT_STATUS, 1, buffer, 0);
}

void setClockSource(uint8_t source) 
{
  writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void setFullScaleGyroRange(uint8_t range) 
{
  writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void setFullScaleAccelRange(uint8_t range) 
{
  writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void setSleepEnabled(int enabled) 
{
  writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, enabled);
}

int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) 
{
  uint8_t count, b;

  if ((count = readBytes(devAddr, regAddr, 1, &b, timeout)) != 0) 
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }

  return count;
}

uint8_t getDeviceID() 
{
  readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer, 0);
  return buffer[0];
}



void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
  uint8_t buffer[16]; 
  int a,b;
  *ax = *ay = *az = *gx = *gy = *gz = 0;
  readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer, 0);
  *ax = (int16_t) (((buffer[0]) << 8) | buffer[1]);
  *ay = (int16_t) (((buffer[2]) << 8) | buffer[3]);
  *az = (int16_t) (((buffer[4]) << 8) | buffer[5]);
  *gx = (int16_t) (((buffer[8]) << 8) | buffer[9]);
  *gy = (int16_t) (((buffer[10]) << 8) | buffer[11]);
  *gz = (int16_t) (((buffer[12]) << 8) | buffer[13]);


}

double getAnglex(double newAnglex, double newRatex, double dtx) 
{
        // KasBot V2  -  ` filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        ratex = newRatex - biasx;
        anglex += dtx * ratex;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        Px[0][0] += dtx * (dtx*Px[1][1] - Px[0][1] - Px[1][0] + Q_anglex);
        Px[0][1] -= dtx * Px[1][1];
        Px[1][0] -= dtx * Px[1][1];
        Px[1][1] += Q_biasx * dtx;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        Sx = Px[0][0] + R_measurex;
        /* Step 5 */
        Kx[0] = Px[0][0] / Sx;
        Kx[1] = Px[1][0] / Sx;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        yx = newAnglex - anglex;
        /* Step 6 */
        anglex += Kx[0] * yx;
        biasx += Kx[1] * yx;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        Px[0][0] -= Kx[0] * Px[0][0];
        Px[0][1] -= Kx[0] * Px[0][1];
        Px[1][0] -= Kx[1] * Px[0][0];
        Px[1][1] -= Kx[1] * Px[0][1];

        return anglex;
}

double getAngley(double newAngley, double newRatey, double dty) 
{
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        ratey = newRatey - biasy;
        angley += dty * ratey;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        Py[0][0] += dty * (dty*Py[1][1] - Py[0][1] - Py[1][0] + Q_angley);
        Py[0][1] -= dty * Py[1][1];
        Py[1][0] -= dty * Py[1][1];
        Py[1][1] += Q_biasy * dty;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        Sy = Py[0][0] + R_measurey;
        /* Step 5 */
        Ky[0] = Py[0][0] / Sy;
        Ky[1] = Py[1][0] / Sy;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        yy = newAngley - angley;
        /* Step 6 */
        angley += Ky[0] * yy;
        biasy += Ky[1] * yy;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        Py[0][0] -= Ky[0] * Py[0][0];
        Py[0][1] -= Ky[0] * Py[0][1];
        Py[1][0] -= Ky[1] * Py[0][0];
        Py[1][1] -= Ky[1] * Py[0][1];

        return angley;
}

void setAngley(double newAngley) // Used to set angle, this should be set as the starting angle
{
  angley = newAngley; 
} 

void setAnglex(double newAnglex) // Used to set angle, this should be set as the starting angle
{
  anglex = newAnglex; 
} 

void kalman_setup()
{
  

  uint8_t tmp;
  
  i2cData[0] = sampling_rate_const; // Set the sample rate to 200Hz - 800Hz/(3+1) = 200Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    Py[0][0] = Px[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    Py[0][1] = Px[0][1] = 0;
    Py[1][0] = Px[1][0] = 0;
    Py[1][1] = Px[1][1] = 0;

  writeBytes(0x68, 0x19, 4, i2cData); // Write to all four registers at once

  tmp = 0x01;
  writeBytes(0x68, 0x6B, 1, &tmp); // PLL with X axis gyroscope reference and disable sleep mode
  readBytes(0x68, 0x3B, 6, i2cData, 0);

  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  setAnglex(roll); // Set starting angle
  setAngley(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = get_time()/400;
}

void calib()
{
  long i, bufy = 0, bufx = 0, bufz = 0, accx = 0, accy = 0, accz = 0;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  for(i = 0 ; i < 200 ; i ++)
  {
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    bufx += gx;
    bufy += gy;
    bufz += gz;
    accx += ax;
    accy += ay;
    accz += az;
  }
  gyrox_offset =  (int16_t)(bufx/(200*1.0));
  gyroy_offset =  (int16_t)(bufy/(200*1.0));
  gyroz_offset =  (int16_t)(bufz/(200*1.0));
  accx_offset  =  (int16_t)(accx/(200*1.0));
  accy_offset  =  (int16_t)(accy/(200*1.0));
  accz_offset  =  (int16_t)(accz/(200*1.0));

}

void init_mpu()
{
  int fde;

  gpio_export(30);
  gpio_set_dir(30, 1);
  gpio_set_value(30,0);

  gpio_export(15);
  gpio_set_dir(15,0);
  fde = open("/sys/class/gpio/gpio15/edge",O_RDWR);
  pwrite(fde,"rising",6,0);
  close(fde);


  setInterruptMode(0); //active high
  setInterruptLatch(1);
  setInterruptLatchClear(0); //clear the interrupt by reading the INT_STATUS register
  setIntDataReadyEnabled(1);

  gpio_export(29);
  gpio_set_dir(29, 1);
  gpio_set_value(29,0);

  devAddr = 0x68;

  setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  setSleepEnabled(0); 

  calib();

  kalman_setup();
}

void kalman_loop(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  double roll, pitch, gyroXrate, gyroYrate;

    gyroX = (double) gx;
    gyroY = (double) gy;


    double dt = (double)((get_time()/400) - timer) / 1000000; // Calculate delta time
    timer = get_time() / 400;

    roll = atan2((double)ay, (double)az) * RAD_TO_DEG;
    pitch = atan((double)-ax / (double)sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    gyroXrate = gyroX / 131.0; // Convert to deg/s
    gyroYrate = gyroY / 131.0; // Convert to deg/s


    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      setAnglex(roll);
      kalAngleX = roll;
      gyroXangle = roll;
    } 
    else
    {
      kalAngleX = getAnglex(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    
    kalAngleY = getAngley(pitch, gyroYrate, dt);
  
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
  
    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;

    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;


   gyroXangle = gyroXangle;
   gyroYangle = gyroYangle;

}

void *mpu_thread()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  int16_t axk, ayk, azk;
  int16_t gxk, gyk, gzk;

  long gtx, gty, gtz;
  long atx, aty, atz;

  int socketfd;
  int len,count,i,j, u;
  struct sockaddr_in socadd;
  int result;
  char ch[256];
  struct pollfd snesorpoll;
  unsigned char buf;

  double dt, timer;
  double angz[2];

  double anglex, angley, anglez;

  
  double scaleaccx[2],scaleaccy[2],scaleaccz[2],scalegyrox,scalegyroy,scalegyroz;
  double velx[2],vely[2],velz[2],posx[2],posy[2],posz[2];

  double anglexp = 0, angleyp = 0;

  double ang;

  double gyroz[2];

  int flag = 0;

  int xc, yc, zc; 

  xc = yc = zc = 0;
  gtx = gty = gtz  = 0;
  atx = aty = atz  = 0;

  memset(ch,'\0',256);

  snesorpoll.fd = open ("/sys/class/gpio/gpio15/value",O_RDWR);
  snesorpoll.events = POLLPRI|POLLERR;
  pread(snesorpoll.fd,&buf,sizeof(buf),0);

  getIntStatus(); 
  

  socketfd = socket(AF_INET, SOCK_STREAM, 0);
  socadd.sin_family = AF_INET;
  socadd.sin_addr.s_addr = inet_addr(IP_AD_UN);
  socadd.sin_port = htons(9999);
  len = sizeof(socadd);

  result = connect(socketfd, (struct sockaddr *)&socadd, len);
  
  if(result == -1)
  {
   printf("CLIENT CONNECTION FAILED");
   terminateflag = 1;
  }

   for( u = 0 ; u < 2 ; u ++)
  {
    scaleaccx[u] = scaleaccy[u] = scaleaccz[u] = velx[u] = vely[u] = velz[u] = posx[u] = posy[u] = posz[u] = 0;
    gyroz[u] = 0;
    angz[u] = 0;
  }

  while(1)
  {
    if(terminateflag == 1)
      break;
    u = 0;
    while(u < 10)
    {
      if ( poll(&snesorpoll,1,-1) != 0)
      {
        pread(snesorpoll.fd,&buf,sizeof(buf),0);
        if (snesorpoll.revents & POLLPRI)
        {
          getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          gyroz[1] += (double) gz;

          gtx += gx;
          gty += gy;
          gtz += gz;

          atx += ax;
          aty += ay;
          atz += az;
          
          u ++;

          getIntStatus(); //clear the interrupt status register
        }
      }
    
    }

    gyroz[1] = (double) (gyroz[1] / 1310.0);

    gxk = (int16_t) (gtx / (10));
    gyk = (int16_t) (gty / (10));
    gzk = (int16_t) (gtz / (10));

    axk = (int16_t) (atx / (10));
    ayk = (int16_t) (aty / (10));
    azk = (int16_t) (atz / (10));

    dt = (double)((get_time()/400) - timer) / 1000000; // Calculate delta time
    timer = get_time() / 400;

    kalman_loop(ax, ay, az, gx, gy, gz);



    ang = (double)((gyroz[0] + (double)((gyroz[1] - gyroz[0])/2))*dt);

    if(( ang < 30 ) && ( ang > -30 ))        
            angz[1] = angz[0] + ang;    

    anglez = (double)((angz[1] - angz[0]) / 2.0);
    anglex = (double)kalAngleX;
    angley = (double)kalAngleY;

    if(anglex - anglexp > -5 && anglex - anglexp < 5)
    {
      anglex = 0;
    }
    else
    {
      
      anglex = (double)((double)((kalAngleX - anglexp)/(dt * 2)) * DEG_TO_RAD);
      anglexp = (double)kalAngleX;  
    }

    if(angley - angleyp > -5 && angley - angleyp < 5)
      angley = 0;
    else
    {
      
      
      angley = (double)((double)((kalAngleY - angleyp)/(dt * 2)) * DEG_TO_RAD);
      angleyp = (double)kalAngleY;
    }
    
    if(flag < 50)
    {
      flag ++;
      snprintf(ch,1024,"%f %f %f %f %f %f\n",0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
      write(socketfd,ch,strlen(ch));
    }
    else
    {
        snprintf(ch,1024,"%f %f %f %f %f %f\n",0.0f,0.0f,0.0f,(float)anglex,(float)anglez,(float)angley);
        write(socketfd,ch,strlen(ch));
    }

    angz[0] = angz[1];
    gyroz[0] = gyroz[1];
    gtx = gty = gtz  = 0;
    atx = aty = atz  = 0;
   
  }
  close(socketfd);
  pthread_exit("Exiting thread");
}

void main()
{

  pthread_t mpu;
  void *thread_result;
  int res;
  char ch;
  init_mpu();

  if (pthread_create(&mpu, NULL, mpu_thread, NULL))
  {
    printf("\nthread failed\n");
  }
  printf("\nENTER A KEY TO TERMINATE\n");
  scanf("%c", &ch);
  terminateflag = 1;

  res = pthread_join(mpu, &thread_result);

  gpio_unexport(29);
  gpio_unexport(30);
  gpio_unexport(15);

  printf("\n program terminated\n");
}