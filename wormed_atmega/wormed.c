//xxxxx/wormed voice touch synth 2017 

// Modes: 0/1/TMS/speak and spell vocal, 2/square wave, 3->11
// wavetables, 12-pulse with length of pulse and frequency, 13-sp0256
// pulse, 14-votrax glottal impulse, 15-impulses from klatt

// license:GPL-2.0+
// copyright-holders: Martin Howse

// NOTES:  ADC is 0 with no finger, do we want noise volume on adcread10(1)

//                  ^TOP^
//  fingers: X- pitch   X-unused
//
//                 X-mode

#define F_CPU 16000000UL 

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "waves.h"


#define BV(bit) (1<<(bit)) // Byte Value => converts bit into a byte value. One at bit location.
#define cbi(reg, bit) reg &= ~(BV(bit)) // Clears the corresponding bit in register reg
#define sbi(reg, bit) reg |= (BV(bit))              // Sets the corresponding bit in register reg


#define FS 8000 // sample rate
#define CHIRP_SIZE 41
#define howmany 10

uint8_t synthPeriod, sample, rate, counter;
volatile uint8_t modus;
uint16_t synthEnergy, location=0;


static uint16_t pitch[howmany];
static float floatrate;
uint8_t pitchindex=0;
uint16_t total=0, average=0;
uint8_t pulsecounter, pulselength;

uint8_t chirp[CHIRP_SIZE] = {0x00,0x2a,0xd4,0x32,0xb2,0x12,0x25,0x14,0x02,0xe1,0xc5,0x02,0x5f,0x5a,0x05,0x0f,0x26,0xfc,0xa5,0xa5,0xd6,0xdd,0xdc,0xfc,0x25,0x2b,0x22,0x21,0x0f,0xff,0xf8,0xee,0xed,0xef,0xf7,0xf6,0xfa,0x00,0x03,0x02,0x01};// tms

uint8_t glottal_wave[9] = {128, 54, 0, 237, 219, 201, 182, 164, 146};// klatt
uint8_t impulsive_source[3] = {128,0,250};
uint8_t rates[16] = {6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t inks[16] =  {1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};


ISR(TIMER1_COMPA_vect) {
  static uint8_t nextPwm, noisy;
  static uint8_t periodCounter;
  static int16_t x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,nsq;
  static uint16_t synthRand = 1;
  static float floatlocation;
  int16_t u0,u1,u2,u3,u4,u5,u6,u7,u8,u9,u10;

  OCR2A= noisy;
  OCR2B = nextPwm;
  sei();

    // Unvoiced source
    synthRand = (synthRand >> 1) ^ ((synthRand & 1) ? 0xB800 : 0);
    u1 = (synthRand & 1) ? synthEnergy : -synthEnergy;
    noisy = (u1>>2)+0x80;
  
  // this is depending on modes

    switch(modus){
    case 0:
    case 1:
    if (periodCounter < synthPeriod) {
      periodCounter++;
    } else {
      periodCounter = 0;
    }
    if (periodCounter < CHIRP_SIZE) {
      u10 = ((chirp[periodCounter]) * (uint32_t) synthEnergy) >> 4; // try to increase volume >>
    } else {
      u10 = 0;
    }
    if (u10 > 511) u10 = 511;
    if (u10 < -512) u10 = -512;
    nextPwm = (u10>>2)+0x80;
    //    nextPwm = (u10)+(512);
    break;
    case 2: // square wave
      nextPwm = 120 + synthPeriod * ((nsq++%synthPeriod)/(synthPeriod/2));
      break;
      // wavetables to test and fill in - all will be slower though so maybe generate faster...
      // sinewave[1024] is uint16, table_kahrs000[160], table_fletcher000[99], plaguetable_simplesir[328], plaguetable_simplesir_002[493], crowtable[142], crowtable_slower[283], triangle_150[420], natural_samples[100]
    case 3: 
      counter--;
      if (counter==0){
      if (location>=99) location-=99;
      nextPwm=pgm_read_byte(&(table_fletcher000[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 4: 
      counter--;
      if (counter==0){
      if (location>=1024) location-=1024;
      nextPwm=pgm_read_byte(&(sinewave[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 5: 
      counter--;
      if (counter==0){
      if (location>=160) location-=160;
      nextPwm=pgm_read_byte(&(table_kahrs000[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 6: 
      counter--;
      if (counter==0){
      if (location>=328) location-=328;
      nextPwm=pgm_read_byte(&(plaguetable_simplesir[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 7: 
      counter--;
      if (counter==0){
      if (location>=142) location-=142;
      nextPwm=pgm_read_byte(&(crowtable[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 8: 
      counter--;
      if (counter==0){
      if (location>=283) location-=283;
      nextPwm=pgm_read_byte(&(crowtable_slower[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 9: 
      counter--;
      if (counter==0){
      if (location>=420) location-=420;
      nextPwm=pgm_read_byte(&(triangle_150[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 10: 
      counter--;
      if (counter==0){
      if (location>=420) location-=420;
      nextPwm=pgm_read_byte(&(sawtooth_150[location])); 
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
    case 11: 
      counter--;
      if (counter==0){
      if (location>=100) location-=100;
      nextPwm=pgm_read_byte(&(natural_samples[location]));
      location+=inks[rate];
      counter=rates[rate];
      }
      break;
      
    case 12: // pulse with length of pulse and frequency
      if (pulsecounter<pulselength){
	pulsecounter++;
	nextPwm=255;
      }
      else
	{
	  if (periodCounter++ <synthPeriod) {
	    nextPwm=0;
	  }
	  else {
	    nextPwm=0;
	    periodCounter=0;
	    pulsecounter=0;
	}
	}
      break;
    case 13: // sp0256 voice pulse -TESTED!
      if (periodCounter <= 0)
	{
	  periodCounter += synthPeriod;
	  nextPwm    = 255; // amp is ???
	} else
	{
	  nextPwm = 128;
	  periodCounter--;
	}
      break;
    case 14: // glottal wave from votrax converted -TESTED!
      nextPwm = periodCounter >= (9 << 2) ? 0 : glottal_wave[periodCounter >> 2];

      // how we change pitch:
      periodCounter = (periodCounter + 1) & 0x7f; // inc

      if(periodCounter == (0x7f ^ synthPeriod)) periodCounter = 0; // 7 bits 128
      break;
    case 15: // impulsive source from nsynth -TESTED!
	if (periodCounter < 3)
	{
		nextPwm = impulsive_source[periodCounter];
	}
	else
	{
		nextPwm = 128;
	}
	if (periodCounter++>synthPeriod) periodCounter=0;
	break;
      
    } // switch
}

void adc_init(void)
{
  	cbi(ADMUX, REFS1); 
	sbi(ADMUX, REFS0); // AVCC
	//	sbi(ADMUX, ADLAR); //8 bits
	sbi(ADCSRA, ADPS2);
	//	sbi(ADCSRA, ADPS1); // change speed here! now div by 64
	//	sbi(ADCSRA, ADPS0); // change speed here!

	sbi(ADCSRA, ADEN);
	DDRC = 0x00;
	PORTC = 0x00;
	// settle on 10 bits
}

void init_all() {

  counter=1;

  // outputs
  
  //  pinMode(3,OUTPUT); //  output for OCR2B - on atmega would be = PD3 OC2B - pin 1
  //  pinMode(11, OUTPUT); //  output for OCR2A = PB3 OC2A - pin 15

  DDRD=0x08;  // PD3
  DDRB=0x08; // PB3  

  // Timer 2 set up as a 62500Hz PWM.
  //
  TCCR2A = _BV(COM2A1) |_BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = 0;
			
  // Timer 1 set up as a 8000Hz sample interrupt
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  TCNT1 = 0;
  OCR1A = F_CPU / FS;
  TIMSK1 = _BV(OCIE1A);  
  synthPeriod=0x50; 
  synthEnergy=14;
  sei();
  // three inputs? one is pitch, bottom one is algo, one is ????
  adc_init();
  // clear pitches
  uint8_t xx;
  for (xx=0;xx<howmany;xx++){
    pitch[xx]=0;
  }  
}

unsigned char adcread(unsigned char channel){
  unsigned char result, high;
  ADMUX &= 0xF8; // clear existing channel selection                
  ADMUX |=(channel & 0x07); // set channel/pin
  ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
  loop_until_bit_is_set(ADCSRA, ADIF); /* Wait for ADIF, will happen soon */
  result=ADCH;
  return result;
}

unsigned int adcread10(short channel){
  unsigned int ADresult;
  ADMUX &= 0xF8; // clear existing channel selection                
    ADMUX |=(channel & 0x07); // set channel/pin
  ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
  loop_until_bit_is_set(ADCSRA, ADIF); /* Wait for ADIF, will happen soon */
  ADresult = ADCL;
  ADresult |= ((int)ADCH) << 8;
  return(ADresult);
}

void main() {

  // TODO: modes smoothed and unsmoothed and with different ranges

  // mode selector is on adcread10(2)

  static uint8_t counterr, mode;
  static int16_t lastsample, sample, lastmaxed, maxed=0;
  init_all();

  while(1){
    // we increment modus on touch
    counterr++;
    
    if (counterr==16){ // might need tweaking but seems to work
      counterr=0;
    if (adcread10(2)>255) mode++;
    modus=mode%16; // 16 modes, could be 32
    }

    //    modus=14; 
    
    // what are the modes?
    switch(modus){
    case 0: // stopped pitch - tms
    // type of glottal from talko, freeze pitch on release
      _delay_ms(10);
      lastsample=sample;
      sample=adcread10(0)>>2;

      if (sample>maxed) {
	maxed=sample;
	lastmaxed=maxed;
      }
  
      if (sample==0 && lastsample==0){
	synthPeriod=128-lastmaxed;
	maxed=0;
      }
      break;
    case 1: // continuous pitch - tms
    case 13:
    case 14:
    case 15:
      //       = 120 + 16 * ((nsq++%16)/8);
      total-=pitch[pitchindex];
      pitch[pitchindex]=adcread10(0);
      total+=pitch[pitchindex++];
      if (pitchindex>=howmany) pitchindex=0;
      average=total/howmany;
      synthPeriod=128-(average>>2); // different modes with different ranges for this
      _delay_ms(10);
      break;
    case 2: // square wave
      total-=pitch[pitchindex];
      pitch[pitchindex]=adcread10(0);
      total+=pitch[pitchindex++];
      if (pitchindex>=howmany) pitchindex=0;
      average=total/howmany;
      synthPeriod=128-(average>>4); // different modes with different ranges for this
      _delay_ms(10);
      break;
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      // wavetable rate
      total-=pitch[pitchindex];
      pitch[pitchindex]=adcread10(0);
      total+=pitch[pitchindex++];
      if (pitchindex>=howmany) pitchindex=0;
      average=total/howmany;
      rate=average>>6; // 10 bits >> 6 = 4 bits
      _delay_ms(10);      
      break;
    case 12: // pulse with 2 params - length of pulse and gap
      synthPeriod=255-(adcread10(0)>>3);
      pulselength=1+(adcread10(1)>>5);
      break;
    }// switch
  }
}

