//Data logger Demonstration using Stalker V2.1 Logs temperature periodically to a file Datalog.csv

//1.Use a FAT16 formatted SD card
//2.Compile and upload the sketch
//3.See if everything works fine using Serial Monitor.
//4.Remove all Serial port code, recompile the sketch and upload.
// This reduces power consumption during battery mode.

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/power.h>
#include <Wire.h>
#include <DS3231.h>
#include <Fat16.h>
#include <Fat16util.h>

#include <SeeedOLED.h>
#include <OneWire.h>


#include <Sensirion.h>        // SHT11 Library from: https://github.com/domhardt/ArduinoLibraries/tree/master/Sensirion
#include <Adafruit_BMP085.h>  // BMP Library from: https://github.com/adafruit/Adafruit-BMP085-Library
#define address 0x31          // I2C address of CO2 Sensor
const uint8_t dataPin  =  16; // SHT11 Data pin connected to Arduino A2
const uint8_t clockPin =  17; // SHT11 Clock pin connected to Arduino A3
Sensirion tempSensor = Sensirion(dataPin, clockPin); //Setup SHT11 Sensor
Adafruit_BMP085 bmp;          // Setup BMP085 Sensor
const int I2Cdelay=100;       // Required delay when reading the CO2 Sensor.

//Assign variables
int co;                 // CO2 stored as an integer
float temperature;      // Temperature stored as a float
float humidity;         // Relative Humidity stored as a float
float dewpoint;         // Dew Point stored as a float
float airpres = 0.0;    // Air Pressure stored as a float
float CalFactor = 8.6;  // Air Pressure Calibration factor - adjsut to allow for alititude
int sampleInterval = 5; // Sample interval in seconds

//Variables used to read CO2 sensor
const int data_bytes = 7;
byte data_buffer [data_bytes];

const int  LED1 = 5;  // LED1 on Arduino pin D5
const int  LED2 = 8;  // LED2 on Arduino pin D8

//The following code is taken from sleep.h as Arduino Software v22 (avrgcc), in w32 does not have the latest sleep.h file
#define sleep_bod_disable() \
{ \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
}


DS3231 RTC; //Create RTC object for DS3231 RTC come Temp Sensor 
char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
//year, month, date, hour, min, sec and week-day(starts from 0 and goes to 6)
//writing any non-existent time-data may interfere with normal operation of the RTC.
//Take care of week-day also.
DateTime dt(2014, 10, 29, 16, 16, 00, 3);

char CH_status_print[][4]=  { "Off","On","Ok","Err" };

static uint8_t prevSecond=0; 
static uint8_t prevMinute=0; 
static DateTime interruptTime;
SdCard card;
Fat16 file;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
    SeeedOled.init();                  //initialize SEEED OLED display
    SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
    SeeedOled.putString("Sim card Error");
  }
  while(1);
}


void setup () 
{
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04; 
     DDRD &=~ 0x04;
     pinMode(4,INPUT);//extern power
   
     Wire.begin();
     Serial.begin(57600);
     RTC.begin();
     bmp.begin();
     //RTC.adjust(dt); //Adjust date-time as defined 'dt' above 
     delay(1000);
     attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     //Enable Interrupt 
     DateTime  start = RTC.now();
     interruptTime = DateTime(start.get() + 10); //Add 10seconds in seconds to start time
     analogReference(INTERNAL); //Read battery status

     Serial.println("Starting");  // Serial feedback
     Serial.println("AirSensor CO2 Shield Test Datalogger");  // Serial feedback
     
     Wire.begin();
     SeeedOled.init();                  //initialize SEEED OLED display
     SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
     SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
     SeeedOled.setPageMode();           //Set addressing mode to Page Mode
     SeeedOled.putString("Starting CO2");
     delay(2000);
     SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
}

void loop () 
{
//   LEDon();             // Turn both LEDs on
    ReadCO2();           // Read CO2 Sensor
    ReadSHT();           // Read SHT11 Sensor
    ReadBMP();           // Read BMP085 Sensor
//    LEDoff();             // Turn both LEDs on
    
    //Battery Charge Status and Voltage reader
    int BatteryValue = analogRead(A7);
    float voltage=BatteryValue * (1.1 / 1024)* (10+2)/2;  //Voltage devider
    unsigned char CHstatus = read_charge_status();//read the charge status
  
    ////////////////////// START : Application or data logging code//////////////////////////////////
    RTC.convertTemperature();          //convert current temperature into registers
    float temp = RTC.getTemperature(); //Read temperature sensor value
    
    DateTime now = RTC.now(); //get the current date-time    
    if((now.second()) !=  prevSecond )
//    if((now.minute()) !=  prevMinute )
    {
    //print only when there is a change
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.date(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(" ");
    Serial.print(weekDay[now.dayOfWeek()]);
    Serial.print(" ");
    Serial.print("Volt: ");
    Serial.print(voltage);
    Serial.print("V, ");
    Serial.print("Charge: ");
    Serial.println(CH_status_print[CHstatus]);
    Serial.print("TempRTC: ");
    Serial.print(temp,2);
    Serial.print(" TempCO2: ");
    Serial.print(temperature,2);
    Serial.write(186);
    Serial.print("C, ");
    Serial.print("Hum: ");
    Serial.print(humidity,2);
    Serial.print(", DP: ");
    Serial.print(dewpoint,2); 
    Serial.print(", CO2: ");
    Serial.print(co);
    Serial.print(", Press: ");
    Serial.println(airpres,2);   
    
    SeeedOled.init();          //clear the screen and set start position to top left corner
    SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
    SeeedOled.setTextXY(0,6);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putNumber(now.year());
    SeeedOled.putString("-");
    SeeedOled.putNumber(now.month());
    SeeedOled.putString("-");
    SeeedOled.putNumber(now.date());   
    SeeedOled.setTextXY(1,8);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putNumber(now.hour());
    SeeedOled.putString(":");
    SeeedOled.putNumber(now.minute());
    SeeedOled.putString(":");
    SeeedOled.putFloat(now.second(),0);   
    SeeedOled.setTextXY(3,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Tmp:");
    SeeedOled.putFloat(temp,1);
    SeeedOled.putString(" io ");
    SeeedOled.putFloat(temperature,1);
    SeeedOled.setTextXY(2,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Co2:");
    SeeedOled.putFloat(co,0);
    SeeedOled.setTextXY(5,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Prs:");
    SeeedOled.putFloat(airpres,2);
    SeeedOled.setTextXY(6,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("DP :");
    SeeedOled.putFloat(dewpoint,2);
    SeeedOled.setTextXY(4,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Hum:");
    SeeedOled.putFloat(humidity,2);
    
    SeeedOled.setTextXY(7,0);          //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Bat:");
    SeeedOled.putFloat(voltage);

    }
     prevSecond = now.second();
     prevMinute = now.minute();
    //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||
    // initialize the SD card
    if (!card.init()) error("card.init");
  
    // initialize a FAT16 volume
    if (!Fat16::init(&card)) error("Fat16::init");
  
    char name[] = "CO2a.CSV";
    // clear write error
    file.writeError = false;
  
    // O_CREAT - create the file if it does not exist
    // O_APPEND - seek to the end of the file prior to each write
    // O_WRITE - open for write
    if (!file.open(name, O_CREAT | O_APPEND | O_WRITE))
        error("error opening file");

    file.print(now.year(), DEC);
    file.print('/');
    file.print(now.month(), DEC);
    file.print('/');
    file.print(now.date(), DEC);
    file.print(' ');
    file.print(now.hour(), DEC);
    file.print(':');
    file.print(now.minute(), DEC);
    file.print(':');
    file.print(now.second(), DEC);
    file.print(',');
    file.print(voltage);
    file.print(',');
    file.print(CH_status_print[CHstatus]);
    file.print(',');
    file.print(temp);
    file.print(',');
    file.print(temperature,2);
    file.print(',');
    file.print(humidity,2);
    file.print(',');
    file.print(dewpoint,2); 
    file.print(',');
    file.print(airpres,2); 
    file.print(',');
    file.println(co);
      

    if (!file.close()) 
        error("error closing file");
    //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||
    RTC.clearINTStatus(); //This function call is  a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    
    
    ////////////////////////END : Application code //////////////////////////////// 
   
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
    //Power Down routines
    cli(); 
    sleep_enable();      // Set sleep enable bit
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
        
    Serial.println("\nSleeping");
    delay(50); //This delay is required to allow print to complete
    //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); //This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)  
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  //This shuts enables ADC, TWI, SPI, Timers and USART
    delay(10); //This delay is required to allow CPU to stabilize
    Serial.println("Awake from sleep");    
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 

unsigned char read_charge_status(void)
{
  unsigned char CH_Status=0;
  unsigned int ADC6=analogRead(6);
  if(ADC6>900)
  {
    CH_Status = 0;//sleeping
  }
  else if(ADC6>550)
  {
    CH_Status = 1;//charging
  }
  else if(ADC6>350)
  {
    CH_Status = 2;//done
  }
  else
  {
    CH_Status = 3;//error
  }
  return CH_Status;
}
  
//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{

  //Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    interruptTime = DateTime(interruptTime.get() + 10);  //decide the time for next interrupt, configure next interrupt  
    Serial.println("Interruption");
}

//Function to read BMP085 Sensor
void ReadBMP()
{
  Serial.println("Read BMP085");  // Serial feedback
  float bmppresval(bmp.readPressure());
  delay (10);
  airpres=((bmppresval/100)+CalFactor);
}

//Function to read SHT11 Sensor
void ReadSHT()
{
  Serial.println("SHT11");
  tempSensor.measure(&temperature, &humidity, &dewpoint);
}

//Function to read CO2 Sensor
void ReadCO2()
{
  Serial.println("Read CO2");
  Wire.beginTransmission(address);
  Wire.write('R');
  Wire.endTransmission();
  
  delay(I2Cdelay);
  Wire.requestFrom(address, data_bytes);
  delay(I2Cdelay);
  while (!Wire.available ());
  for (int i = 0; i < data_bytes; ++i)
  {data_buffer [i] = Wire.read();
  delay(I2Cdelay);
  }
  
  for (int i=0;i<7;i++)
  {
   byte c = data_buffer [i];
   if (i==1) co = c;
   if (i==2) co = (co << 8) | c;
  }
}

//Function to turn both LEDs on
void LEDon()
{
    Serial.println("LED on");
    digitalWrite(LED1, HIGH);
    delay(250);
    digitalWrite(LED2, HIGH);
}


//Function to turn both LEDs off
void LEDoff()
{
    Serial.println("LED off");
    digitalWrite(LED2, LOW);
    delay(250);
    digitalWrite(LED1, LOW);
}
