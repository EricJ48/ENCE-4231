#define BUTTON_2  2  // Inc Timer
#define BUTTON_1  3  // Start/Stop Timer and Stop Buzzer
#define GREEN_LED 4
#define RED_LED   5
#define BUZZER    6

#define DATA      9 // DS
#define LATCH     8 // ST_CP
#define CLOCK     7 // SH_CP

#define DIGIT_4   10
#define DIGIT_3   11
#define DIGIT_2   12
#define DIGIT_1   13
#define LED 13
#define BUFF_SIZE 20

// Serial port variables
char  gIncomingChar;
char  gCommsMsgBuff[BUFF_SIZE];
int   iBuff = 0;
byte  gPackageFlag = 0;
byte  gProcessDataFlag = 0;

// 7-Seg Display Variables
unsigned char gtable[]=
{0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c
,0x39,0x5e,0x79,0x71,0x00};
byte gCurrentDigit;

// Volatile Variables
volatile unsigned char gISRFlag1   = 0;
volatile unsigned char gISRFlag2   = 0;
volatile unsigned char gBuzzerFlag = 0;

// Timer Variables
#define DEFAULT_COUNT 30     // default value is 30secs
volatile int  gCount        = DEFAULT_COUNT;
unsigned char gTimerRunning = 0; 

unsigned int gReloadTimer1 = 62500; // corresponds to 1 second
byte         gReloadTimer2 = 10;  // display refresh time
unsigned int gReloadTimer3 = 100;   // corresponds to 0.4ms

/**
 * @brief Setup peripherals and timers
 * @param
 * @return
 */
void setup() {
  // LEDs Pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // LEDs -> Timer Stopped
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  // Button Pins
  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), buttonISR1, RISING);
  pinMode(BUTTON_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), buttonISR2, RISING);

  // Buzer Pins
  pinMode(BUZZER, OUTPUT);

  // Buzzer -> Off
  digitalWrite(BUZZER,LOW);

  // 7-Seg Display
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Shift Register Pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);

  dispOff();  // turn off the display

  // Initialize Timer1 (16bit) -> Used for clock
  // Speed of Timer1 = 16MHz/256 = 62.5 KHz
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = gReloadTimer1; // compare match register 16MHz/256
  TCCR1B |= (1<<WGM12);   // CTC mode
  // Start Timer by setting the prescaler -> done using the start button
  //TCCR1B |= (1<<CS12);    // 256 prescaler 
  TIMSK1 |= (1<<OCIE1A);  // enable timer compare interrupt
  interrupts();

  // Initialize Timer2 (8bit) -> Used to refresh display
  // Speed of Timer2 = 16MHz/1024 = 15.625 KHz
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = gReloadTimer2;                     // max value 2^8 - 1 = 255
  TCCR2A |= (1<<WGM21);
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // 1204 prescaler
  TIMSK2 |= (1<<OCIE2A);
  interrupts();                               // enable all interrupts
}

  // Initialize Timer3 (16bit) -> Used for Serial Comms
  // Speed of Timer3 = 16MHz/64 = 250 KHz
  TCCR3A = 0;
  TCCR3B = 0;
  OCR3A = gReloadTimer1;            // max value 2^16 - 1 = 65535
  TCCR3A |= (1<<WGM31);
  TCCR3B = (1<<CS31) | (1<<CS30);   // 64 prescaler
  TIMSK3 |= (1<<OCIE3A);

/**
 * @brief Shifts the bits through the shift register
 * @param num
 * @param dp
 * @return
 */
void display(unsigned char num, unsigned char dp)
{
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, gtable[num] | (dp<<7));
  digitalWrite(LATCH, HIGH);
}

/**
 * @brief Turns the 7-seg display off
 * @param
 * @return
 */
void dispOff()
{
   digitalWrite(DIGIT_1, HIGH);
   digitalWrite(DIGIT_2, HIGH);
   digitalWrite(DIGIT_3, HIGH);
   digitalWrite(DIGIT_4, HIGH);
}

/**
 * @brief Button 2 ISR
 * @param
 * @return
 */
void buttonISR2()
{
  // Increment Clock
  gCount++;
}

/**
 * @brief Button 1 ISR
 * @param
 * @return
 */
void buttonISR1()
{ 
  // Set ISR Flag
  gISRFlag1 = 1;
}

/**
 * @brief Timer 2 ISR
 * @param TIMER2_COMPA_vect
 * @return
 */
ISR(TIMER2_COMPA_vect)   // Timer2 interrupt service routine (ISR)
{
  dispOff();  // turn off the display
 
  switch (gCurrentDigit)
  {
    case 1: //0x:xx
      display( int((gCount/60) / 10) % 6, 0 );   // prepare to display digit 1 (most left)
      digitalWrite(DIGIT_1, LOW);  // turn on digit 1
      break;
 
    case 2: //x0:xx
      display( int(gCount / 60) % 10, 1 );   // prepare to display digit 2
      digitalWrite(DIGIT_2, LOW);     // turn on digit 2
      break;
 
    case 3: //xx:0x
      display( (gCount / 10) % 6, 0 );   // prepare to display digit 3
      digitalWrite(DIGIT_3, LOW);    // turn on digit 3
      break;
 
    case 4: //xx:x0
      display(gCount % 10, 0); // prepare to display digit 4 (most right)
      digitalWrite(DIGIT_4, LOW);  // turn on digit 4
      break;

    default:
      break;
  }
 
  gCurrentDigit = (gCurrentDigit % 4) + 1;
}

/**
 * @brief Timer 1 ISR
 * @param TIMER1_COMPA_vect
 * @return
 */
ISR(TIMER1_COMPA_vect)  // Timer1 interrupt service routine (ISR)
{
  gCount--;

  if(gCount == 0)
  {
      // Stop Timer
      stopTimer1();
      
      // Raise Alarm
      gBuzzerFlag = 1;
      gTimerRunning = 0;
  }
}

ISR(TIMER1_COMPA_vect)  // Timer1 interrupt service routine (ISR)
{
  if(Serial.available()>0)
  {
    gISRFlag2 = 1;
  }
}

/**
 * @brief Stop Timer 1
 * @param
 * @return
 */
void stopTimer1()
{
  // Stop Timer
  TCCR1B &= 0b11111000; // stop clock
  TIMSK1 = 0; // cancel clock timer interrupt
}

/**
 * @brief Start Timer 1
 * @param
 * @return
 */
void startTimer1()
{
  // Start Timer
  TCCR1B |= (1<<CS12);    // 256 prescaler 
  TIMSK1 |= (1<<OCIE1A);  // enable timer compare interrupt
}

/**
 * @brief Turn On Buzzer
 * @param
 * @return
 */
void activeBuzzer()
{
  unsigned char i;
  unsigned char sleepTime = 1; // ms
  
  for(i=0;i<100;i++)
  {
    digitalWrite(BUZZER,HIGH);
    delay(sleepTime);//wait for 1ms
    digitalWrite(BUZZER,LOW);
    delay(sleepTime);//wait for 1ms
  }
}

char compareArray(char a[], char b[], int size)
{
  int i;
  char result = 1;  // default: the arrays are equal
  
  for(i = 0; i<size; i++)
  {
    if(a[i]!=b[i])
    {
      result = 0;
      break;
    }
  }
  return result;
}

/**
 * @brief Main Loop
 * @param
 * @return
 */
void loop() 
{
  // Attend Button 2 ISR
  if(gISRFlag1 == 1)
  {
    // Reset ISR Flag
    gISRFlag1 = 0;

    if(gTimerRunning == 0)
    {
      // Start Timer
      gTimerRunning = 1;

      if(gCount == 0)
        gCount = DEFAULT_COUNT;

      if(gBuzzerFlag == 1)
      {
        gBuzzerFlag = 0;

        // LEDs -> Timer Stopped
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
      }
      else
      {
        startTimer1();
        // LEDs -> Timer Running
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
      }
    }
    else
    {
      // Stop Timer
      stopTimer1();
      gTimerRunning = 0;

      // LEDs -> Timer Running
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
    }
  }

  // Attend gBuzzerFlag
  if(gBuzzerFlag == 1)
  {
    // Make Noise...
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    activeBuzzer();
  }

    char  auxMsgBuff[BUFF_SIZE];
  int auxCount = 0;
  unsigned char auxDigit = '0';
  
  // Attend Timer1 flag - receive commands through serial
  if(gISRFlag2 == 1)
  {    
    // Reset ISR Flag
    gISRFlag2 = 0;

    // Read serial
    gIncomingChar = Serial.read();

    // If normal character from package
    if(gPackageFlag == 1)
    {
      gCommsMsgBuff[iBuff] = gIncomingChar;
      iBuff++;

      // Safety mechanism in case "\n" is never sent
      if(iBuff == BUFF_SIZE)
      {
        gPackageFlag = 0;
        gProcessDataFlag = 1;
      }
    }

    // If start of the package
    if(gIncomingChar == '$')
    {    
      gPackageFlag = 1;  // Signal start of package
      
      // Clear Buffer
      for(int i=0; i<BUFF_SIZE; i++)
      {
        gCommsMsgBuff[i] = 0;
      }

      // set gCommsMsgBuff Index to zero
      iBuff = 0;
    }

    // If end of package
    if( (gIncomingChar == '\n') && (gPackageFlag == 1) )
    {
      // Signal end of package
      gPackageFlag = 0;
      gProcessDataFlag = 1;
    }
  }

  // Process serial commands
  if(gProcessDataFlag == 1)
  {
    gProcessDataFlag = 0;
    
    if(compareArray(gCommsMsgBuff, "STR", 3) == 1)
    {
      // Start timer function
      startTimer1();
    }
  
    if(compareArray(gCommsMsgBuff, "STP", 3) == 1)
    {
      // Stop timer function
      stopTimer1();
    }

    if(compareArray(gCommsMsgBuff, "GET", 3) == 1)
    {
      // Send clock status
      Serial.print("$");
      Serial.print(digit1);
      Serial.print(digit2);
      Serial.print(":");
      Serial.print(digit3);
      Serial.print(digit4);
      Serial.print("\n");
    }
    if(compareArray(gCommsMsgBuff, "INC", 3) == 1)
    {
      gCount++;
    }
  }
}
