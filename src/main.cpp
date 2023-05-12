#include <mbed.h>

/*******************************************/
//DEFINES
/*******************************************/
// #define BUTTON_STATE HAL_GPIO_ReadPin(GPIOB, PA_0)
// // #define BUTTON_STATE HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
// #define LED_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
// #define LED_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)


#define CTRL_REG1 0x20  // Controls the operating mode and data rate of the L3GD20 gyro sensor
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG  0x27
#define WHO_AM_I  0x0F

#define STATUS_REG_BIT_3  0x08
#define STATUS_REG_BIT_7  0x80
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

#define SPI_READ  0x80
#define SPI_WRITE 0x00
#define SPI_DUMMY 0x00
#define NXT_REG 0x40

/* Angular Rate Modes */
#define ANG_RATE_MODE1  0 // +/-250 dps
#define ANG_RATE_MODE2  1 // +/-500 dps
#define ANG_RATE_MODE3  2 // +/-1000 dps
#define ANG_RATE_MODE   4 // +/-2000 dps

#define Pi  3.142

/* Factor to convert Angles to radians */
#define ANG_TO_RAD  0.0174533  // Pi/180

/*******************************************/
//PIN DECLARATIONS
/*******************************************/
SPI spi(PF_9, PF_8, PF_7);
DigitalOut cs(PC_1);

/*******************************************/
//GLOBAL VARIABLES
/*******************************************/
int RegRW;
Timer tim;
Ticker t;
volatile short xAxis = 0, yAxis = 0, zAxis = 0;
volatile int xAxis_rad = 0, yAxis_rad = 0, zAxis_rad = 0;
volatile bool FetchSamples = 0;
volatile int AngularMode = 0;
int arr_index = 0;

volatile int xAxis_arr[200], yAxis_arr[200], zAxis_arr[200];
float avrg_AnglVel_x, avrg_AnglVel_y, avrg_AnglVel_z;
float lat_Vel;
float dist_cov;


InterruptIn button(PA_0);
Timeout debounceTimeout;
volatile bool debounceFlag;
volatile bool buttonPressed;

// debouncing clear
void debounce_clear()
{
    debounceFlag = false;
}


// button interrupt handler
void button_isr()
{
    if (debounceFlag)
    {
        return;
    }
    debounceFlag = true;
    // TODO: replace Timeout with Ticker, since Timeout has been deprecated
    debounceTimeout.attach(&debounce_clear, 0.05); // Set the timeout to 50 ms
    buttonPressed = true;
}

void init_button()
{
    button.fall(&button_isr);
}

/*******************************************/
//FUNCTIONS
/*******************************************/
void sysStall()
{
  /* Exception handler could be written */
}

short GetAxesFromGyroSensor(int Reg_L_Addr, int Reg_H_Addr)
{
      unsigned int Axis;
      int RegRW;
      volatile unsigned char status;
      volatile bool NewData = false;

      /* Read status register to check if data is ready to be
         to be read from the gyro sensor */

      while (!NewData)
      {
        cs = 0;
        RegRW = SPI_READ | STATUS_REG;
        spi.write(RegRW);
        status = spi.write(SPI_DUMMY);
        cs = 1;

        if((status & STATUS_REG_BIT_3) > 0x00)
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

/******************************************************/
//MAIN FUNCTION
/******************************************************/
int main() {
  init_button();

  /* Chip must be deselected at first */
  cs = 1;

  spi.format(8,3);
  spi.frequency(1000000);

  /* Read and validate who am i register for gyro */
  cs = 0;
  RegRW = SPI_READ | WHO_AM_I;
  spi.write(RegRW);   /* To read from the who am I register */
  int whoami = spi.write(SPI_DUMMY);
  printf("Value in WHOAMI register = 0x%X\n", whoami);
  cs = 1;

//   wait_us(10000);

  if(whoami != 0xD3)
  {
    printf("We are not referring to the gyro sensor L3GD0 on STM32 Discovery board\n");
    sysStall();
  }

  /* Set up control register to change power mode from Power down to Normal and enable
     all the x, y and z axes */
  cs = 0;
  RegRW = SPI_WRITE | CTRL_REG1;
  spi.write(RegRW);
  RegRW = 0x0F;   // To bring the Gyro sensor out of power down mode and enable all the three axes
  spi.write(RegRW);
  cs = 1;
  
  tim.start();
  t.attach(&FetchAxesValues, 500ms);

        if(buttonPressed)
        {
            /* Button is pressed */
             while(1) {

                
                FetchSamples = 0;
                /* To get X Axis values */
                xAxis = (int)GetAxesFromGyroSensor(OUT_X_L, OUT_X_H);
                /* To get Y Axis values */
                yAxis = (int)GetAxesFromGyroSensor(OUT_Y_L, OUT_Y_H);
                /* To get Z Axis values */
                zAxis = (int)GetAxesFromGyroSensor(OUT_Z_L, OUT_Z_H);

                    xAxis_rad = xAxis * ANG_TO_RAD * 0.01775;
                    yAxis_rad = yAxis * ANG_TO_RAD * 0.01775;
                    zAxis_rad = zAxis * ANG_TO_RAD * 0.01775;
                    xAxis_arr[arr_index] = xAxis_rad;
                    yAxis_arr[arr_index] = yAxis_rad;
                    zAxis_arr[arr_index] = zAxis_rad;
                    arr_index++;

                    printf("x- %d\n",xAxis_rad);
                    printf("y- %d\n",yAxis_rad);
                    printf("z- %d\n",zAxis_rad);
                    printf("\n\n");

                    // if(arr_index==200) break;
                    rtos::ThisThread::sleep_for(100ms);
                
             }
    }
        else
      {
          /* do nothing*/
      }

    return 0;
}
