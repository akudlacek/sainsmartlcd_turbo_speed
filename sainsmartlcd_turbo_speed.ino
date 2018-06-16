//hardware circuit list for Arduino Uno Rev3
//    PORT        Arduino label    dir    processor pin     peripheral           function
//  PB0  0x01           8           I          14        PCINT0/CLKO/ICP1       LCD_RS
//  PB1  0x02          ~9           I          15        OC1A/PCINT1            LCD_EN
//  PB2  0x04         ~10           I          16        SS/OC1B/PCINT2         LCD_BACKLIGHT
//  PB3  0x08         ~11           I          17        MOSI/OC2A/PCINT3      
//  PB4  0x10          12           I          18        MISO/PCINT4           
//  PB5  0x20          13           I          19        SCK/PCINT5             

//  PC0  0x01          A0           I          23        ADC0/PCINT8            KEYPAD
//  PC1  0x02          A1           I          24        ADC1/PCINT9           
//  PC2  0x04          A2           I          25        ADC2/PCINT10          
//  PC3  0x08          A3           I          26        ADC3/PCINT11          
//  PC4  0x10          A4           I          27        ADC4/SDA/PCINT12
//  PC5  0x20          A5           I          28        ADC5/SCL/PCINT13

//  PD0  0x01       RX<-0           O          2         PCINT16/RXD           ************
//  PD1  0x02       TX->1           O          3         PCINT17/TXD           ************
//  PD2  0x04           2           I          4         PCINT18/INT0          speed_sensor_input
//  PD3  0x08          ~3           I          5         PCINT19/OC2B/INT1    
//  PD4  0x10           4           I          6         PCINT20/XCK/T0        LCD_D4
//  PD5  0x20          ~5           I          11        PCINT21/OC0B/T1       LCD_D5
//  PD6  0x40          ~6           I          12        PCINT22/OC0A/AIN0     LCD_D6
//  PD7  0x80           7           I          13        PCINT23/AIN1          LCD_D7

#include <LiquidCrystal.h>


/******************************************************************************
  Defines
******************************************************************************/
/*Turbo and sensor config parameters*/
#define NUMBER_OF_TEETH       14 //number of fins on turbo
#define SENSOR_FACTOR         8  //number of fin passes for one pulse output of sensor
#define HZ_TO_RPM             (((double)SENSOR_FACTOR * (double)60.0) / (double)NUMBER_OF_TEETH)
#define REDLINE          113633  //ACTUALL REDLINE IS 122374 for 87S72 (AirWerks S300SX-E) 13009097006  13009097047

/*
 * External interrupt pin for speed sensor
 * see following URL for which pins are compatable
 * https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ 
 */
#define speed_sensor_input 2 //external_interrupt_0_pin_2

/*SainSmart LCD Keypad pinout*/
typedef enum
{
  LCD_RS = 8,
  LCD_EN = 9,
  LCD_D4 = 4,
  LCD_D5 = 5,
  LCD_D6 = 6,
  LCD_D7 = 7,
  LCD_BACKLIGHT = 10,
  KEYPAD = 0
} sainsmart_lcd_keypad_pins_t;


/******************************************************************************
  Variables
******************************************************************************/
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
volatile uint16_t pulse_count = 0;
volatile uint16_t pulses_per_sec = 0;
volatile uint8_t sys_tick_s = 0;
volatile uint8_t last_sys_tick_s = 0;
uint32_t turbo_speed = 0;
uint32_t turbo_speed_max = 0;

byte up_custom_lcd_char[] = {
  B01010,
  B01010,
  B01110,
  B00000,
  B01110,
  B01010,
  B01110,
  B01000
};


/******************************************************************************
  Local Prototypes
******************************************************************************/
void drvr_tc1_init(uint16_t freq_ocr_val);
void ext_interrupt_handler(void);


/******************************************************************************
   \brief setup

   \note
******************************************************************************/
void setup()
{
  /*Init serial*/
  Serial.begin(9600);

  /*Init external interrupt for speed senseor*/
  pinMode(speed_sensor_input, INPUT);
  attachInterrupt(digitalPinToInterrupt(speed_sensor_input), ext_interrupt_handler, RISING);

  /*Init timer1 16-BIT so 1 second interrupts*/
  drvr_tc1_init(62499);

  /*Init SainsSmart LCD Keypad Shield*/
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  lcd.begin(16, 2);           // set up the LCD's number of columns and rows:
  lcd.clear();
  lcd.createChar(0, up_custom_lcd_char);

  /*Enable interrupts*/
  interrupts(); //enable interrupts

  /*Send configuration to serial port*/
  Serial.print("NUMBER_OF_TEETH: ");
  Serial.println(NUMBER_OF_TEETH);

  Serial.print("SENSOR_FACTOR: ");
  Serial.println(SENSOR_FACTOR);

  Serial.print("HZ_TO_RPM: ");
  Serial.println(HZ_TO_RPM);

  Serial.print("REDLINE: ");
  Serial.println(REDLINE);
}

/******************************************************************************
   \brief Main loop

   \note
******************************************************************************/
void loop()
{
  
  if(last_sys_tick_s != sys_tick_s)
  {
    /*Get turbo speed from Hz*/
    turbo_speed = (uint32_t)((double)pulses_per_sec * HZ_TO_RPM);

    /*LCD - RPM*/
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RPM: ");
    lcd.print(turbo_speed);

    /*LCD - ovrspeed flag*/
    if(turbo_speed > REDLINE)
    {
      lcd.setCursor(13, 0);
      lcd.print("OVR");
    }

    /*LCD - max RPM*/
    lcd.setCursor(0, 1);
    if(turbo_speed > turbo_speed_max)
    {
      turbo_speed_max = turbo_speed;
      lcd.print("MAX:");
      lcd.write(byte(0));
    }
    else
    {
      lcd.print("MAX: ");
    }
    lcd.print(turbo_speed_max);

    /*LCD - raw frequency*/
    lcd.setCursor(12, 1);
    lcd.print(pulses_per_sec);

    last_sys_tick_s = sys_tick_s;
  }
}


/**************************************************************************************************
                                        LOCAL FUNCTIONS
**************************************************************************************************/
/******************************************************************************
   \brief TC1 - 16-bit Timer/Counter1 with PWM init

   \note PB2/OC1B is PWM, OC1A is freq control
******************************************************************************/
void drvr_tc1_init(uint16_t freq_ocr_val)
{
  //clkI/O/256 (From prescaler)
  //CTC
  //Normal port operation, OC1A/OC1B disconnected.
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS12);
  TCCR1C = 0;

  //set freq
  OCR1AH = (uint8_t)(freq_ocr_val >> 8);
  OCR1AL = (uint8_t)(freq_ocr_val);

  TIMSK1 = _BV(1); //Output Compare A Match Interrupt Enable
}


/**************************************************************************************************
                                            HANDLERS
**************************************************************************************************/
/******************************************************************************
   \brief external interrupt handler

   \note
******************************************************************************/
void ext_interrupt_handler(void)
{
  pulse_count++;
}

/******************************************************************************
   \brief Compare Match ISR (OC1A)

   \note
******************************************************************************/
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
  pulses_per_sec = pulse_count;
  pulse_count = 0;
  sys_tick_s++;
}


