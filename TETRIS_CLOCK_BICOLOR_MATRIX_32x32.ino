//************************************************************************************************************//
  
//********************* TETRIS CLOCK ON BICOLOR LED MATRIX USING BIT ANGLE MODULATION METHOD ********************//
- Using RTC DS3231.
- Arduino Mega 2560.
- Refer Tetris Template from: https://github.com/toblum/esp_p10_tetris_clock

//************************************************************************************************************//
#include <SPI.h>
#include "Wire.h"
#include "TetrisNumbers.h"

// REAL PIN ON ARDUINO

#define blank_pin       3   // BLANK PIN - 74HC595
#define latch_pin       2   // LATCH PIN - 74HC595
#define clock_pin       52  // CLOCK PIN - 74HC595
#define data_pin        51  // DATA PIN - 74HC595

#define RowA_Pin        22  // A PIN - 74HC238
#define RowB_Pin        24  // B PIN - 74HC238
#define RowC_Pin        26  // C PIN - 74HC238
#define RowD_Pin        28  // D PIN - 74HC238
//#define OE_Pin        30  // ENABLE OUTPUT PIN - 74HC238

//MAPPING TO PORT

#define Blank_Pin_Bit   5 // PORTE bit 5 - PE5
#define Latch_Pin_Bit   4 // PORTE bit 4 - PE4

#define RowA_Pin_Bit    0 // PORTA bit 0 - PA0
#define RowB_Pin_Bit    2 // PORTA bit 2 - PA2
#define RowC_Pin_Bit    4 // PORTA bit 4 - PA4
#define RowD_Pin_Bit    6 // PORTA bit 6 - PA6
#define OE_Pin_Bit      7 // PORTC bit 7 - PC7


// *************************************************For Tetris Clock*****************************************//

fall_instr current_fall;
fall_instr fallen_block;
byte s0, s1, m0, m1, h0, h1;

// *************************************************For DS3231******************************************//
#define DS3231_I2C_ADDRESS 0x68
unsigned long samplingtime = 0;
unsigned long samplingtime1 = 0;
byte ssecond, sminute, shour, sdayOfWeek, sdayOfMonth, smonth, syear;
byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;

//****************************************************For BAM**************************************************//
#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G (256 colours)

const  byte Size_Y = 32;    //Number of LEDS in Y axis (Top to Bottom)
const  byte Size_X = 32;    //Number of LEDs in X axis (Left to Right)

byte red[4][128];
byte green[4][128];

int level=0;                //keeps track of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

//************************************************************************************************************//

void setup()
{
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  noInterrupts();
  
  TCCR1A = B00000000;
  TCCR1B = B00001011;
  TIMSK1 = B00000010;
  OCR1A = 15;
  
  pinMode(latch_pin, OUTPUT);
  //pinMode(blank_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);
  pinMode(clock_pin, OUTPUT);
  
  pinMode(RowA_Pin, OUTPUT);
  pinMode(RowB_Pin, OUTPUT);
  pinMode(RowC_Pin, OUTPUT);
  pinMode(RowD_Pin, OUTPUT);
  //pinMode(OE_Pin, OUTPUT);
  
  //digitalWrite(OE_Pin, HIGH);
  SPI.begin();
  interrupts();
  Wire.begin();
  clearfast();
}

void loop()
{

  Seconds_Divider();     
  Display_Tetris_S1();
  Display_Tetris_S0();
  Display_Tetris_M1();
  Display_Tetris_M0();
  Display_Tetris_H1();
  Display_Tetris_H0();

}

void LED(int X, int Y, int R, int G)
{
  X = constrain(X, 0, Size_X - 1); 
  Y = constrain(Y, 0, Size_Y - 1);
  
  R = constrain(R, 0, (1 << BAM_RESOLUTION) - 1);
  G = constrain(G, 0, (1 << BAM_RESOLUTION) - 1); 

  int WhichByte = int(Y*4+ X/8);
  int WhichBit = 7-(X%8);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][WhichByte], WhichBit, bitRead(R, BAM));

    bitWrite(green[BAM][WhichByte], WhichBit, bitRead(G, BAM));
  }

}

void rowScan(byte row)
{
  
  if (row & 0x01) PORTA |= _BV(RowA_Pin_Bit);   //PORTA |= _BV(0)
    else PORTA &= ~_BV(RowA_Pin_Bit);           //PORTA &= ~_BV(0)
  
  if (row & 0x02) PORTA |= _BV(RowB_Pin_Bit);   //PORTA |= _BV(2)
    else PORTA &= ~_BV(RowB_Pin_Bit);           //PORTA &= ~_BV(2)

  if (row & 0x04) PORTA |= _BV(RowC_Pin_Bit);   //PORTA |= _BV(4)
    else PORTA &= ~_BV(RowC_Pin_Bit);           //PORTA &= ~_BV(4)

  if (row & 0x08) PORTA |= _BV(RowD_Pin_Bit);   //PORTA |= _BV(6)
    else PORTA &= ~_BV(RowD_Pin_Bit);           //PORTA &= ~_BV(6)
}

ISR(TIMER1_COMPA_vect){
  
PORTE |= ((1<<Blank_Pin_Bit));                  // Set BLANK PIN high - 74HC595
//PORTC |= _BV(OE_Pin_Bit);                     // Set OUTPUT ENABLE high - 74HC238

if(BAM_Counter==4)
BAM_Bit++;
else
if(BAM_Counter==12)
BAM_Bit++;
else
if(BAM_Counter==28)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:

      //Red
      
        myTransfer(red[0][level + 0]);
        myTransfer(red[0][level + 1]);
        myTransfer(red[0][level + 2]);
        myTransfer(red[0][level + 3]);
        myTransfer(red[0][level + 64]);
        myTransfer(red[0][level + 65]);
        myTransfer(red[0][level + 66]);
        myTransfer(red[0][level + 67]);

      //Green
        
        myTransfer(green[0][level + 0]);
        myTransfer(green[0][level + 1]);
        myTransfer(green[0][level + 2]);
        myTransfer(green[0][level + 3]);
        myTransfer(green[0][level + 64]);
        myTransfer(green[0][level + 65]);
        myTransfer(green[0][level + 66]);
        myTransfer(green[0][level + 67]);

      break;
    case 1:
       
      //Red

        myTransfer(red[1][level + 0]);
        myTransfer(red[1][level + 1]);
        myTransfer(red[1][level + 2]);
        myTransfer(red[1][level + 3]);  

        myTransfer(red[1][level + 64]);
        myTransfer(red[1][level + 65]);
        myTransfer(red[1][level + 66]);
        myTransfer(red[1][level + 67]);             
      //Green

        myTransfer(green[1][level + 0]);
        myTransfer(green[1][level + 1]);
        myTransfer(green[1][level + 2]);
        myTransfer(green[1][level + 3]);
        
        myTransfer(green[1][level + 64]);
        myTransfer(green[1][level + 65]);
        myTransfer(green[1][level + 66]);
        myTransfer(green[1][level + 67]);
        
      break;
    case 2:
      
      //Red

        myTransfer(red[2][level + 0]);
        myTransfer(red[2][level + 1]);
        myTransfer(red[2][level + 2]);
        myTransfer(red[2][level + 3]);
        
        myTransfer(red[2][level + 64]);
        myTransfer(red[2][level + 65]);
        myTransfer(red[2][level + 66]);
        myTransfer(red[2][level + 67]);
                      
       //Green

        myTransfer(green[2][level + 0]);
        myTransfer(green[2][level + 1]);
        myTransfer(green[2][level + 2]);
        myTransfer(green[2][level + 3]);
        
        myTransfer(green[2][level + 64]);
        myTransfer(green[2][level + 65]);
        myTransfer(green[2][level + 66]);
        myTransfer(green[2][level + 67]);

      break;
    case 3:
      //Red

        myTransfer(red[3][level + 0]);
        myTransfer(red[3][level + 1]);
        myTransfer(red[3][level + 2]);
        myTransfer(red[3][level + 3]); 
               
        myTransfer(red[3][level + 64]);
        myTransfer(red[3][level + 65]);
        myTransfer(red[3][level + 66]);
        myTransfer(red[3][level + 67]);    

      //Green

        myTransfer(green[3][level + 0]);
        myTransfer(green[3][level + 1]);
        myTransfer(green[3][level + 2]);
        myTransfer(green[3][level + 3]);
              
        myTransfer(green[3][level + 64]);
        myTransfer(green[3][level + 65]);
        myTransfer(green[3][level + 66]);
        myTransfer(green[3][level + 67]);
        
  if(BAM_Counter==60){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

PORTE |= 1<<Latch_Pin_Bit;
PORTE &= ~(1<<Latch_Pin_Bit);
PORTE &= ~(1<<Blank_Pin_Bit);       // Set BLANK PIN low - 74HC595
//PORTC &= ~_BV(OE_Pin_Bit);        // Set OUTPUT ENABLE low - 74HC238

row++;
level = row<<2;
if(row==16)
row=0;
if(level==64)
level=0;

DDRE |= _BV (Blank_Pin_Bit);    // pinMode (blank_pin, OUTPUT);
//DDRC |= _BV (OE_Pin_Bit);     // pinMode (OE_Pin, OUTPUT);

}

inline static uint8_t myTransfer(uint8_t C_data){
  SPDR = C_data;
  asm volatile("nop"); 
  asm volatile("nop");  
}


void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 128);
    memset(green, 0, sizeof(green[0][0]) * 4 * 128);
}

void fillTable(byte R, byte G)
{
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<32; y++)
      {
        LED(x, y, R, G);
      }
    }
}

void fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color)
{
    for (uint16_t x = x1; x <= x2; x++) {
        for (uint16_t y = y1; y <= y2; y++) {
            LED(x,y, color.red, color.green);      
        }
    }
}

// Convert decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}

// Convert binary coded decimal to decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}


void ReadDS3231Time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // Set DS3231 register pointer to 00h
  Wire.endTransmission();

  // Request seven bytes of data from DS3231 starting from register 00h
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  *second = (Wire.read() & 0x7f);
  *minute = (Wire.read());
  *hour = (Wire.read() & 0x3f);
 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// *********************************************************************
// Draws a brick shape at a given position
// *********************************************************************
void drawShape(int blocktype, Color color, int x_pos, int y_pos, int num_rot)
{
  // Square
  if (blocktype == 0)
  {
    LED(x_pos, y_pos, color.red, color.green);
    LED(x_pos + 1, y_pos, color.red, color.green);
    LED(x_pos, y_pos - 1, color.red, color.green);
    LED(x_pos + 1, y_pos - 1, color.red, color.green);
  }

  // L-Shape
  if (blocktype == 1)
  {
    if (num_rot == 0)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 1)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 2, y_pos - 1, color.red, color.green);
    }
    if (num_rot == 2)
    {
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 2, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 3)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos - 1, color.red, color.green);
    }
  }

  // L-Shape (reverse)
  if (blocktype == 2)
  {
    if (num_rot == 0)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 1)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
    }
    if (num_rot == 2)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
      LED(x_pos + 1, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 3)
    {
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 2, y_pos - 1, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
    }
  }

  // I-Shape
  if (blocktype == 3)
  {
    if (num_rot == 0 || num_rot == 2)
    { // Horizontal
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
      LED(x_pos + 3, y_pos, color.red, color.green);
    }
    if (num_rot == 1 || num_rot == 3)
    { // Vertical
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
      LED(x_pos, y_pos - 3, color.red, color.green);
    }
  }

  // S-Shape
  if (blocktype == 4)
  {
    if (num_rot == 0 || num_rot == 2)
    {
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 1 || num_rot == 3)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 2, y_pos - 1, color.red, color.green);
    }
  }

  // S-Shape (reversed)
  if (blocktype == 5)
  {
    if (num_rot == 0 || num_rot == 2)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 2, color.red, color.green);
    }
    if (num_rot == 1 || num_rot == 3)
    {
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
    }
  }

  // Half cross
  if (blocktype == 6)
  {
    if (num_rot == 0)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos + 2, y_pos, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
    }
    if (num_rot == 1)
    {
      LED(x_pos, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos, y_pos - 2, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
    }
    if (num_rot == 2)
    {
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 2, y_pos - 1, color.red, color.green);
    }
    if (num_rot == 3)
    {
      LED(x_pos + 1, y_pos, color.red, color.green);
      LED(x_pos, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 1, color.red, color.green);
      LED(x_pos + 1, y_pos - 2, color.red, color.green);
    }
  }

   // Corner-Shape 
   if (blocktype == 7)
   {
     if (num_rot == 0)
     {
       LED(x_pos, y_pos, color.red, color.green);
       LED(x_pos + 1, y_pos, color.red, color.green);
       LED(x_pos, y_pos - 1, color.red, color.green);
     }
     if (num_rot == 1)
     {
       LED(x_pos, y_pos, color.red, color.green);
       LED(x_pos, y_pos - 1, color.red, color.green);
       LED(x_pos + 1, y_pos - 1, color.red, color.green);
     }
     if (num_rot == 2)
     {
       LED(x_pos + 1 , y_pos, color.red, color.green);
       LED(x_pos + 1 , y_pos - 1, color.red, color.green);
       LED(x_pos, y_pos - 1, color.red, color.green);
     }
     if (num_rot == 3)
     {
       LED(x_pos, y_pos, color.red, color.green);
       LED(x_pos + 1, y_pos , color.red, color.green);
       LED(x_pos + 1, y_pos - 1, color.red, color.green);
     }
   }
}

void setNumState(int index, int value, int x_shift)
{
    if(index < TETRIS_MAX_NUMBERS) {
        numstates[index].num_to_draw = value;
        numstates[index].x_shift = x_shift;
        numstates[index].fallindex = 0;
        numstates[index].blockindex = 0;
    }
}

// *********************************************************************
// Helper function that return the number of bricks for a given number
// *********************************************************************
int getBocksizeByNum(int num)
{
  if (num == 0)
  {
    return SIZE_NUM_0;
  }
  if (num == 1)
  {
    return SIZE_NUM_1;
  }
  if (num == 2)
  {
    return SIZE_NUM_2;
  }
  if (num == 3)
  {
    return SIZE_NUM_3;
  }
  if (num == 4)
  {
    return SIZE_NUM_4;
  }
  if (num == 5)
  {
    return SIZE_NUM_5;
  }
  if (num == 6)
  {
    return SIZE_NUM_6;
  }
  if (num == 7)
  {
    return SIZE_NUM_7;
  }
  if (num == 8)
  {
    return SIZE_NUM_8;
  }
  if (num == 9)
  {
    return SIZE_NUM_9;
  }
}


void clearnumber(int number) // clear led matrix_8x8 number at top position 0 1 2 3 and avoid to clear "Second Divider"
{
for (unsigned char j=0; j<16; j++)
  {
    int numberx = number + j*4 ;
    if (number==1 && (numberx==33 || numberx==37 || numberx==49 || numberx==53))
      {
        red[0][numberx] &= 0x01;
        red[1][numberx] &= 0x01;
        red[2][numberx] &= 0x01;
        red[3][numberx] &= 0x01;
        green[0][numberx] &= 0x01;
        green[1][numberx] &= 0x01;
        green[2][numberx] &= 0x01;
        green[3][numberx] &= 0x01;
      }
    else if (number==2 && (numberx==34 || numberx==38 || numberx==50 || numberx==54))
      {
        red[0][numberx] &= 0x80;
        red[1][numberx] &= 0x80;
        red[2][numberx] &= 0x80;
        red[3][numberx] &= 0x80;
        green[0][numberx] &= 0x80;
        green[1][numberx] &= 0x80;
        green[2][numberx] &= 0x80;
        green[3][numberx] &= 0x80;
      }
    else
      {
        red[0][numberx] = 0;
        red[1][numberx] = 0;
        red[2][numberx] = 0;
        red[3][numberx] = 0;
        green[0][numberx] = 0;
        green[1][numberx] = 0;
        green[2][numberx] = 0;
        green[3][numberx] = 0;
      }
  }
}

void clearnumber32(int number) // clear led matrix_8x8 number at botton position 4 5 6 7
{
for (unsigned char j=0; j<16; j++)
  {
      int numberx = 60 + number + j*4 ;
      red[0][numberx] = 0;
      red[1][numberx] = 0;
      red[2][numberx] = 0;
      red[3][numberx] = 0;
      green[0][numberx] = 0;
      green[1][numberx] = 0;
      green[2][numberx] = 0;
      green[3][numberx] = 0;
  }
}

// Clear all top 16x32
void clearfast_tophalf()
{
for (unsigned char j=0; j<64; j++)
{
red[0][j] = 0;
red[1][j] = 0;
red[2][j] = 0;
red[3][j] = 0;
green[0][j] = 0;
green[1][j] = 0;
green[2][j] = 0;
green[3][j] = 0;
}
}

// Clear all bottom 16x32
void clearfast_bottomhalf()
{
for (unsigned char j=64; j<128; j++)
{
red[0][j] = 0;
red[1][j] = 0;
red[2][j] = 0;
red[3][j] = 0;
green[0][j] = 0;
green[1][j] = 0;
green[2][j] = 0;
green[3][j] = 0;
}
}


// Minutes/ Seconds divider (blinking)
void Seconds_Divider()
  {
      fillRectangle(15, 8, 16, 9, orangecolor);
      fillRectangle(15, 12, 16, 13, orangecolor);
  }
void Seconds_DividerClear()
  {
    fillRectangle(15, 8, 16, 9, clearcolor);
    fillRectangle(15, 12, 16, 13, clearcolor);

  }

// *********************************************************************
// Main function that handles the drawing of all numbers
// *********************************************************************
void drawNumbers(int numpos)
{
    // Draw falling shape
    if (numstates[numpos].blockindex < getBocksizeByNum(numstates[numpos].num_to_draw))
    {
      current_fall = getFallinstrByNum(numstates[numpos].num_to_draw, numstates[numpos].blockindex);

      // Handle variations of rotations
      uint8_t rotations = current_fall.num_rot;
      if (rotations == 1)
      {
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 2))
        {
          rotations = 0;
        }
      }
      if (rotations == 2)
      {
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 3))
        {
          rotations = 0;
        }
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 3 * 2))
        {
          rotations = 1;
        }
      }
      if (rotations == 3)
      {
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 4))
        {
          rotations = 0;
        }
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 4 * 2))
        {
          rotations = 1;
        }
        if (numstates[numpos].fallindex < (int)(current_fall.y_stop / 4 * 3))
        {
          rotations = 2;
        }
      }
      
      drawShape(current_fall.blocktype, current_fall.color, current_fall.x_pos + numstates[numpos].x_shift, numstates[numpos].fallindex - 1, rotations);            
      numstates[numpos].fallindex++;
      
      if (numstates[numpos].fallindex > current_fall.y_stop)
      {
        numstates[numpos].fallindex = 0;
        numstates[numpos].blockindex++;
      }
    }

    // Draw already dropped shapes
    if (numstates[numpos].blockindex > 0)
    {
      
      for (int i = 0; i < numstates[numpos].blockindex; i++)
      {
        fallen_block = getFallinstrByNum(numstates[numpos].num_to_draw, i);
        drawShape(fallen_block.blocktype, fallen_block.color, fallen_block.x_pos + numstates[numpos].x_shift, fallen_block.y_stop - 1, fallen_block.num_rot);
        
      }
    }
}


void drawNumbers1632(int numpos)
{
    // Draw falling shape
    if (numstates[numpos].blockindex < getBocksizeByNum(numstates[numpos].num_to_draw))
    {
      current_fall = getFallinstrByNum(numstates[numpos].num_to_draw, numstates[numpos].blockindex);

      // Handle variations of rotations
      uint8_t rotations = current_fall.num_rot;
      if (rotations == 1)
      {
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 2))
        {
          rotations = 0;
        }
      }
      if (rotations == 2)
      {
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 3))
        {
          rotations = 0;
        }
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 3 * 2))
        {
          rotations = 1;
        }
      }
      if (rotations == 3)
      {
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 4))
        {
          rotations = 0;
        }
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 4 * 2))
        {
          rotations = 1;
        }
        if ((numstates[numpos].fallindex) < (int)((current_fall.y_stop + 16) / 4 * 3))
        {
          rotations = 2;
        }
      }
      
      drawShape(current_fall.blocktype, current_fall.color, current_fall.x_pos + numstates[numpos].x_shift, numstates[numpos].fallindex - 1 , rotations);
           
      numstates[numpos].fallindex ++;

      if ((numstates[numpos].fallindex) > (current_fall.y_stop + 16))
      {
        numstates[numpos].fallindex = 16;
        numstates[numpos].blockindex++;
      }
    }

    // Draw already dropped shapes
    if (numstates[numpos].blockindex > 0)
    {
      
      for (int i = 0; i < numstates[numpos].blockindex; i++)
      {
        fallen_block = getFallinstrByNum(numstates[numpos].num_to_draw, i);
        drawShape(fallen_block.blocktype, fallen_block.color, fallen_block.x_pos + numstates[numpos].x_shift, fallen_block.y_stop - 1 + 16, fallen_block.num_rot);
        
      }
    }
}

// *********************************************************************
// Main function that shows the tetris animation clock on LED 32x32
// *********************************************************************

void Display_Tetris_S1()
{
clearnumber(3);
  if ( (unsigned long) (micros() - samplingtime) > 3)
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);
    s1 = (csecond & 0x0f);
    if (s1!=numstates[3].num_to_draw)  
      {
        if (s1==0 || s1==5)
          {
            setNumState(3, s1, 26);          
            numstates[3].num_to_draw = s1;   
          }                       
        }                             
    drawNumbers(3);
    samplingtime = micros();       
  }
}

void Display_Tetris_S0()
{
  clearnumber(2);
  if ( (unsigned long) (micros() - samplingtime) > 3  )
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);
    s0 = ((csecond >> 4) & 0x0f);
    if (s0!=numstates[2].num_to_draw)  
      {
        setNumState(2, s0, 18);          
        numstates[2].num_to_draw = s0;                          
      }
    drawNumbers(2);                      
    samplingtime = micros();
  }
}


void Display_Tetris_M1()
{
  clearnumber(1);
  if ( (unsigned long) (micros() - samplingtime) > 3  )
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);

    m1 = (cminute & 0x0f);                               
    if (m1!=numstates[1].num_to_draw)  
      {
        setNumState(1, m1, 8);          
        numstates[1].num_to_draw = m1;                          
      }
    drawNumbers(1);      
    samplingtime = micros();
  }
}

void Display_Tetris_M0()
{
  clearnumber(0);
  if ( (unsigned long) (micros() - samplingtime) > 3  )
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);    
    m0 = ((cminute >> 4) & 0x0f);
    if (m0!=numstates[0].num_to_draw)  
      {
        setNumState(0, m0, 0);          
        numstates[0].num_to_draw = m0;                          
      }
    drawNumbers(0);                                       
    samplingtime = micros();
  }
}


void Display_Tetris_H1()
{
  clearnumber32(6);
  clearnumber32(7);
  if ( (unsigned long) (micros() - samplingtime) > 3)
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);
    
    h1 = (chour & 0x0f);
    if (h1!=numstates[6].num_to_draw)  
      {
        setNumState(6, h1, 16);          
        numstates[6].num_to_draw = h1;                        
      }                             
    drawNumbers1632(6);
    samplingtime = micros();       
  }
}

void Display_Tetris_H0()
{
  clearnumber32(5);
  if ( (unsigned long) (micros() - samplingtime) > 3  )
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear);
    
        h0 = ((chour >> 4) & 0x0f);
        if (h0!=numstates[5].num_to_draw)  
        {
          setNumState(5, h0, 8);          
          numstates[5].num_to_draw = h0;                          
        }
    drawNumbers1632(5);                      
    samplingtime = micros();
  }
}
