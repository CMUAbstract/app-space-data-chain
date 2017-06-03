#include <msp430.h>

#include <libio/console.h>
#include <libmsp/sleep.h>

  // Table 6-62: ADC12 calibration for 1.2v reference
#define TLV_CAL30 ((int *)(0x01A1A))
#define TLV_CAL85 ((int *)(0x01A1C))

void init_temp_sensor() {
  return;
}

// Returns temperature in degrees C (approx range -40deg - 85deg)
int read_temperature_sensor() {
  ADC12CTL0 &= ~ADC12ENC;           // Disable conversions

  ADC12CTL3 |= ADC12TCMAP;
  ADC12CTL1 = ADC12SHP;
  ADC12CTL2 = ADC12RES_2;
  ADC12MCTL0 = ADC12VRSEL_1 | BIT4 | BIT3 | BIT2 | BIT1;
  ADC12CTL0 |= ADC12SHT03 | ADC12ON;

  while( REFCTL0 & REFGENBUSY );

  REFCTL0 = REFVSEL_0 | REFON;

  // Wait for REF to settle
  msp_sleep(3); // ~5ms @ ACLK/64 cycles => 32768Hz

  ADC12CTL0 |= ADC12ENC;                         // Enable conversions
  ADC12CTL0 |= ADC12SC;                   // Start conversion
  while (ADC12CTL1 & ADC12BUSY) ;
  
  int sample = ADC12MEM0;

  ADC12CTL0 &= ~ADC12ENC;           // Disable conversions
  ADC12CTL0 &= ~(ADC12ON);  // Shutdown ADC12
  REFCTL0 &= ~REFON;

  int cal30 = *TLV_CAL30;
  int cal85 = *TLV_CAL85;
  int tempC = (sample - cal30) * 55 / (cal85 - cal30) + 30;

  LOG("[temp] sample=%i => T=%i\r\n", sample, tempC);

  return tempC;
}
