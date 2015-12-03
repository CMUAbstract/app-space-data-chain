#include <msp430.h>
#define TF_TEMP_15_30 2982
#define TF_TEMP_15_85 3515
#define TLVStruct(x)  *(&(((int*)TLV_ADC12_1_TAG_)[x+1]))

#define INTERNAL1V2 (ADC12VRSEL_1 | REFON | REFVSEL_0)
#define REF_MASK 0x31
#define REFV_MASK 0x0F00
unsigned int CAL_ADC_12T30;
unsigned int CAL_ADC_12T85;
float CAL_TEMP_SCALE_100X;

unsigned char sensor_busy = 0;

void init_temp_sensor() {

  return;

}

unsigned int get_calibrated_adc () {

  signed long tmptemp;

  unsigned short *tlv_cal30 = (unsigned short *)(0x01A1A);
  unsigned short *tlv_cal85 = (unsigned short *)(0x01A1C);
  
  ADC12CTL0 &= ~ADC12ENC;           // Disable conversions

  ADC12CTL3 |= ADC12TCMAP;
  ADC12CTL1 = ADC12SHP;
  ADC12CTL2 = ADC12RES_2;
  ADC12MCTL0 = ADC12VRSEL_1 | BIT4 | BIT3 | BIT2 | BIT1;
  ADC12CTL0 |= ADC12SHT03 | ADC12ON;

  while( REFCTL0 & REFGENBUSY );

  REFCTL0 = REFVSEL_0 | REFON;

  __delay_cycles(1000);                      // Delay for Ref to settle

  ADC12CTL0 |= ADC12ENC;                         // Enable conversions
  ADC12CTL0 |= ADC12SC;                   // Start conversion
  while (ADC12CTL1 & ADC12BUSY) ;
  
  tmptemp = (signed long) ADC12MEM0;

  ADC12CTL0 &= ~ADC12ENC;           // Disable conversions
  ADC12CTL0 &= ~(ADC12ON);  // Shutdown ADC12
  REFCTL0 &= ~REFON;

  //#define ADC_TO_dC(_v) (3000 + (int)((85 - 30) * ((100L * ((int)(_v) - t30)) / (long)(t85 - t30))))
  signed long tempC = (signed long) tmptemp - (signed long) *tlv_cal30;
  tempC *= 550;
  tempC /= (signed long) (*tlv_cal85 - *tlv_cal30);
  tempC += 300;

  return (signed short) tempC;

}

signed short read_temperature_sensor() {

  signed short temp = get_calibrated_adc();

  return temp;

}

