#include <mbed.h>
#include "stdlib.h"
#include "drivers/LCD_DISCO_F429ZI.h"

// PIN Configurations are as MOSI, MISO, SCLK
// SPI spi(PE_6, PE_5, PE_2);
// DigitalOut slave_slct(PE_4);
LCD_DISCO_F429ZI lcd;

// int main() {

//   // put your setup code here, to run once:

//   while(1) {
//     // put your main code here, to run repeatedly:
//   }
// }


SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2, PullDown);

#define OUT_X_L 0x28
// register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
// configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
// register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
// configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
// register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
// configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;

// button interrupt
InterruptIn button(PA_0);
Timeout debounceTimeout;
volatile bool debounceFlag;
volatile bool buttonPressed;
volatile bool isButtonPressDelayLong;
uint8_t buttonCount = 0; // TODO remove
volatile time_t lastButtonPressTime;
volatile time_t currentButtonPressTime;

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
    buttonPressed = false;
}


void displayMessage1()
{
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DrawHLine(0, LINE(2), 240);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"!ALERT MESSAGE!", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(9), (uint8_t *)" DEFLATION RATE WAS", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"NOT STEADY", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(11), (uint8_t *)"PLEASE PRESS THE", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"RESET BUTTON TO", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(13), (uint8_t *)"START AGAIN", CENTER_MODE);
    BSP_LCD_DrawRect(14, LINE(8), 212, LINE(7));
}


    


void displayMessage2()
{
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DrawHLine(0, LINE(2), 240);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"!ALERT MESSAGE!", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(9), (uint8_t *)"THESHOLD VALUE", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"IS REACHED", CENTER_MODE);
    BSP_LCD_DrawRect(20, LINE(8), 200, LINE(4));
    wait_us(3000000);
    // lcd.Clear(LCD_COLOR_BLACK);
    // lcd.SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawHLine(0, LINE(2), 240);
    // lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"!ALERT MESSAGE!", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"START DECREASING", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(17), (uint8_t *)"THE PRESSURE", CENTER_MODE);
    BSP_LCD_DrawRect(20, LINE(8), 200, LINE(4));
    wait_us(3000000);
}




int main() {

  init_button();

  // put your setup code here, to run once:


  while(1) {
    // put your main code here, to run repeatedly:

    if (buttonPressed) {
      printf("> button pressed.\n");
      displayMessage1();
      buttonPressed = false;
    }
    
  }
}