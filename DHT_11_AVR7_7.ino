/*
 * ATmega16_DHT11_Project_File.c
 *
 * http://www.electronicwings.com
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "LCD16x2_4bit.h"
#define DHT11_PIN 4
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

#define LCD16X2_4BIT_H_

#define  F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

#define LCD_DPRT PORTD
#define LCD_DDDR DDRD
#define LCD_RS 2
#define LCD_EN 3

void lcdcommand(unsigned char cmnd)
{
  LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd & 0xf0);   /* SEND COMMAND TO DATA PORT */
  LCD_DPRT &= ~ (1<<LCD_RS);            /* RS = 0 FOR COMMAND */
  LCD_DPRT |= (1<<LCD_EN);            /* EN = 1 FOR H TO L PULSE */
  _delay_us(1);                 /* WAIT FOR MAKE ENABLE WIDE */
  LCD_DPRT &= ~(1<<LCD_EN);           /* EN = 0 FOR H TO L PULSE */
  _delay_us(100);                 /* WAIT FOR MAKE ENABLE WIDE */
  
  LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd << 4);   /* SEND COMMAND TO DATA PORT */
  LCD_DPRT |= (1<<LCD_EN);            /* EN = 1 FOR H TO L PULSE */
  _delay_us(1);                 /* WAIT FOR MAKE ENABLE WIDE */
  LCD_DPRT &= ~(1<<LCD_EN);           /* EN = 0 FOR H TO L PULSE */
  _delay_ms(2);                 /* WAIT FOR MAKE ENABLE WIDE */
}

void lcddata(unsigned char data)
{
  LCD_DPRT = (LCD_DPRT & 0x0f)|(data & 0xf0);   /* SEND DATA TO DATA PORT */
  LCD_DPRT |= (1<<LCD_RS);            /* MAKE RS = 1 FOR DATA */
  LCD_DPRT |= (1<<LCD_EN);            /* EN=0 FOR H TO L PULSE */
  _delay_us(1);                 /* WAIT FOR MAKE ENABLE WIDE */
  LCD_DPRT &= ~(1<<LCD_EN);           /* EN = 0 FOR H TO L PULSE */
  _delay_us(100);                 /* WAIT FOR MAKE ENABLE WIDE */
  
  LCD_DPRT = (LCD_DPRT & 0x0f)|(data << 4);   /*  */
  LCD_DPRT |= (1<<LCD_EN);            /* EN=0 FOR H TO L PULSE*/
  _delay_us(1);                 /* WAIT FOR MAKE ENABLE WIDE*/
  LCD_DPRT &= ~(1<<LCD_EN);           /* EN = 0 FOR H TO L PULSE*/
  _delay_ms(2);                 /* WAIT FOR MAKE ENABLE WIDE*/
}

void lcdinit()
{
  LCD_DDDR = 0xFF;
  _delay_ms(200);                 /* WAIT FOR SOME TIME */
  lcdcommand(0x33);
  lcdcommand(0x32);               /* SEND $32 FOR INIT OT 0X02 */
  lcdcommand(0x28);               /* INIT. LCD 2 LINE, 5 X 7 MATRIX */
  lcdcommand(0x0C);               /* DISPLAY ON CURSOR ON */
  lcdcommand(0x01);               /* LCD CLEAR */
  _delay_ms(2);
  lcdcommand(0x82);               /* SHIFT CURSOR TO WRITE */
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
  unsigned char firstcharadd[]={0x80, 0xC0};
  lcdcommand(firstcharadd[y] + x);
}

void lcd_print(char *str)
{
  unsigned char i=0;
  while (str[i] |= 0)
  {
    lcddata(str[i]);
    i++;
  }
}

void lcd_clear()
{
  lcdcommand(0x01);
  _delay_ms(2);
}

void Request()            /* Microcontroller send start pulse or request */
{
  DDRB |= (1<<DHT11_PIN);
  PORTB &= ~(1<<DHT11_PIN);   /* set to low pin */
  _delay_ms(20);          /* wait for 20ms */
  PORTB |= (1<<DHT11_PIN);    /* set to high pin */
}

void Response()           /* receive response from DHT11 */
{
  DDRB &= ~(1<<DHT11_PIN);
  while(PINB & (1<<DHT11_PIN));
  while((PINB & (1<<DHT11_PIN))==0);
  while(PINB & (1<<DHT11_PIN));
}

uint8_t Receive_data()              /* receive data */
{ 
  for (int q=0; q<8; q++)
  {
    while((PINB & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
    _delay_us(30);
    if(PINB & (1<<DHT11_PIN))       /* if high pulse is greater than 30ms */
    c = (c<<1)|(0x01);            /* then its logic HIGH */
    else                  /* otherwise its logic LOW */
    c = (c<<1);
    while(PINB & (1<<DHT11_PIN));
  }
  return c;
}

int main(void)
{ 
  char data[5];
  lcdinit();          /* initialize LCD */
  lcd_clear();        /* clear LCD */
  lcd_gotoxy(0,0);      /* enter column and row position */
  lcd_print("Humidity =");
  lcd_gotoxy(0,1);
  lcd_print("Temp = ");
  int RH;
  int Temp;
    while(1)
  { 
    Request();        /* send start pulse */
    Response();       /* receive response */
    I_RH=Receive_data();  /* store first eight bit in I_RH */
    D_RH=Receive_data();  /* store next eight bit in D_RH */
    I_Temp=Receive_data();  /* store next eight bit in I_Temp */
    D_Temp=Receive_data();  /* store next eight bit in D_Temp */
    CheckSum=Receive_data();/* store next eight bit in CheckSum */
    
    if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
    {
      lcd_gotoxy(0,0);
      //lcd_print("Error");
    }
    
    else
    { 
      RH= (I_RH * 256 + D_RH )/ 10.0; 
      itoa(RH,data,10);
      lcd_gotoxy(10,0);
      lcd_print(data);
      lcd_print(".");
      
      itoa(D_RH,data,10);
      lcd_print(data);
      lcd_print("%");
      Temp = (I_Temp * 256 + D_Temp)/ 10.0;
      itoa(Temp,data,10);
      lcd_gotoxy(6,1);
      lcd_print(data);
      lcd_print(".");
      
      itoa(D_Temp,data,10);
      lcd_print(data);
      lcddata(0xDF);
      lcd_print("C ");
      
      itoa(CheckSum,data,10);
      lcd_print(data);
      lcd_print(" ");
    }
        
  _delay_ms(500);
  } 
}
