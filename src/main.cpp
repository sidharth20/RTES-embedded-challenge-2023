#include <mbed.h>

/*******************************************/
// DEFINES
/*******************************************/
#define BUTTON_STATE HAL_GPIO_ReadPin(GPIOB, PA_0)
// #define BUTTON_STATE HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
#define LED_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

#define CTRL_REG1 0x20 // Controls the operating mode and data rate of the L3GD20 gyro sensor
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG 0x27
#define WHO_AM_I 0x0F

#define STATUS_REG_BIT_3 0x08
#define STATUS_REG_BIT_7 0x80
#define CTRL_1_DR 0x40
#define CTRL_1_BW 0x30

/* Out data registers for X, Y and Z axes:
X corresponds to Pitch angle
Y corresponds to Roll angle and
Z corresponds to Yaw angle */
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define SPI_READ 0x80
#define SPI_WRITE 0x00
#define SPI_DUMMY 0x00
#define NXT_REG 0x40

/* Angular Rate Modes */
#define ANG_RATE_MODE1 0 // +/-250 dps
#define ANG_RATE_MODE2 1 // +/-500 dps
#define ANG_RATE_MODE3 2 // +/-1000 dps
#define ANG_RATE_MODE 4  // +/-2000 dps

#define Pi 3.142

/* Factor to convert Angles to radians */
#define ANG_TO_RAD 0.0174533 // Pi/180
#define THRESHOLD 60
#define RESOLUTION 80
/*******************************************/
// PIN DECLARATIONS
/*******************************************/
SPI spi(PF_9, PF_8, PF_7);
DigitalOut cs(PC_1);

/*******************************************/
// GLOBAL VARIABLES
/*******************************************/
int RegRW;
Timer tim;
Ticker t;
volatile short xAxis = 0, yAxis = 0, zAxis = 0;
volatile float xAxis_rad = 0, yAxis_rad = 0, zAxis_rad = 0;
volatile bool FetchSamples = 0;
volatile int AngularMode = 0;
int arr_index = 0;

volatile float xAxis_arr_base[200], yAxis_arr_base[200], zAxis_arr_base[200];
volatile float xAxis_arr_new[200], yAxis_arr_new[200], zAxis_arr_new[200];
float avrg_AnglVel_x, avrg_AnglVel_y, avrg_AnglVel_z;
float lat_Vel;
float dist_cov;
/*******************************************/
// FUNCTIONS
/*******************************************/
void sysStall()
{
}

short GetAxesFromGyroSensor(int Reg_L_Addr, int Reg_H_Addr)
{
  unsigned int Axis;
  int RegRW;
  volatile unsigned char status;
  volatile bool NewData = false;

  while (!NewData)
  {
    cs = 0;
    RegRW = SPI_READ | STATUS_REG;
    spi.write(RegRW);
    status = spi.write(SPI_DUMMY);
    cs = 1;

    if ((status & STATUS_REG_BIT_3) > 0x00)
    {
      NewData = true;
    }
  }

  cs = 0;
  RegRW = SPI_READ | Reg_H_Addr;
  spi.write(RegRW);
  Axis = spi.write(SPI_DUMMY);
  cs = 1;

  cs = 0;
  RegRW = SPI_READ | Reg_L_Addr;
  Axis <<= 8;
  spi.write(RegRW);
  Axis |= spi.write(SPI_DUMMY);
  cs = 1;

  return Axis;
}

void FetchAxesValues()
{
  FetchSamples = 1;
}

float getX()
{
  xAxis = (int)GetAxesFromGyroSensor(OUT_X_L, OUT_X_H);
  xAxis_rad = xAxis * ANG_TO_RAD * 0.01775;
  return xAxis_rad;
}

float getY()
{
  yAxis = (int)GetAxesFromGyroSensor(OUT_Y_L, OUT_Y_H);
  yAxis_rad = yAxis * ANG_TO_RAD * 0.01775;
  return yAxis_rad;
}

float getZ()
{
  zAxis = (int)GetAxesFromGyroSensor(OUT_Z_L, OUT_Z_H);
  zAxis_rad = zAxis * ANG_TO_RAD * 0.01775;
  return zAxis_rad;
}

bool findMatch()
{
  // If the error between the recorded and input sequence is within the tolerance, return true.
  int x_loss = 0;
  int y_loss = 0;
  int z_loss = 0;
  for (uint32_t i = 0; i < RESOLUTION; i++)
  {
    // x_loss += pow((xAxis_arr_base[i] - xAxis_arr_new[i]), 2);
    // y_loss += pow((yAxis_arr_base[i] - yAxis_arr_new[i]), 2);
    // z_loss += pow((zAxis_arr_base[i] - zAxis_arr_new[i]), 2);

    x_loss += (xAxis_arr_base[i] - xAxis_arr_new[i]);
    y_loss += (yAxis_arr_base[i] - yAxis_arr_new[i]);
    z_loss += (zAxis_arr_base[i] - zAxis_arr_new[i]);
  }
  printf("%d, %d, %d\n", x_loss, y_loss, z_loss);
  if (x_loss > THRESHOLD || y_loss > THRESHOLD || z_loss > THRESHOLD)
    return false;
  return true;
}

int main()
{
  cs = 1;

  spi.format(8, 3);
  spi.frequency(1000000);

  cs = 0;
  RegRW = SPI_READ | WHO_AM_I;
  spi.write(RegRW);
  int whoami = spi.write(SPI_DUMMY);
  printf("Value in WHOAMI register = 0x%X\n", whoami);
  cs = 1;

  if (whoami != 0xD3)
  {
    printf("We are not referring to the gyro sensor L3GD0 on STM32 Discovery board\n");
    sysStall();
  }

  cs = 0;
  RegRW = SPI_WRITE | CTRL_REG1;
  spi.write(RegRW);
  RegRW = 0x0F;
  spi.write(RegRW);
  cs = 1;

  tim.start();
  t.attach(&FetchAxesValues, 500ms);
  int recording_count = 2;
  bool result = false;

  while (1)
  {
    float gx = getX();
    float gy = getY();
    float gz = getZ();
    FetchSamples = 0;
    if (recording_count < 3 && ((int)gx != 0 || (int)gy != 0 || (int)gz != 0))
    {

      arr_index = 0;
      while (arr_index < RESOLUTION)
      {
        gx = getX();
        gy = getY();
        gz = getZ();

        xAxis_arr_base[arr_index] += gx;
        yAxis_arr_base[arr_index] += gy;
        zAxis_arr_base[arr_index] += gz;

        arr_index++;
        //  add the values to array
        // record 200 values and break
        printf("x- %f\n", gx);
        printf("y- %f\n", gy);
        printf("z- %f\n", gz);
        printf("\n\n");
        rtos::ThisThread::sleep_for(25ms);
      }
      printf("gesture %d recorded\n", recording_count + 1);
      printf("wait for 5 seconds\n");
      recording_count++;
      if (recording_count < 3)
      {
        rtos::ThisThread::sleep_for(2000ms);
        printf("start next recording\n");
      }
    }
    else if (recording_count >= 3)
    {
      for (int i = 0; i < RESOLUTION; i++)
      {
        xAxis_arr_base[i] /= 3;
        yAxis_arr_base[i] /= 3;
        zAxis_arr_base[i] /= 3;
      }
      printf(" the gesture is recorded and saved, please wait for the next command\n");
      rtos::ThisThread::sleep_for(3000ms);
      printf("The device is in recognition mode, Please make a gesture\n");
      break;
    }
    else
    {
      // write relavent print statements
      printf("The device is in recording mode, please record the %d \n", recording_count);
    }
  }
  printf("Make a gesture\n");
  while (1)
  {
    float gx = getX();
    float gy = getY();
    float gz = getZ();
    FetchSamples = 0;
    if ((int)gx != 0 || (int)gy != 0 || (int)gz != 0)
    {

      arr_index = 0;
      while (arr_index < RESOLUTION)
      {
        gx = getX();
        gy = getY();
        gz = getZ();

        xAxis_arr_new[arr_index] = gx;
        yAxis_arr_new[arr_index] = gy;
        zAxis_arr_new[arr_index] = gz;

        arr_index++;
        //  add the values to array
        // record 200 values and break
        printf("x- %f\n", gx);
        printf("y- %f\n", gy);
        printf("z- %f\n", gz);
        printf("\n\n");
        rtos::ThisThread::sleep_for(25ms);
      }
      printf("Gesture recorded\n");
      printf("Matching the gesture\n");
      result = findMatch();
      if (result)
      {
        printf("gesture matched, unlocked\n");
        // break;
      }
      else
      {
        printf("gesture not matched, try again in 2 seconds\n");
      }
    }
  }

  return 0;
}
