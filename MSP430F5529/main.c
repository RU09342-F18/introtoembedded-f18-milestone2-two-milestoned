/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - ADC12, Sample A0, Set P1.0 if A0 > 0.5*AVcc
//
//   Description: A single sample is made on A0 with reference to AVcc.
//   Software sets ADC12SC to start sample and conversion - ADC12SC
//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
//
//                MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//     Vin -->|P6.0/CB0/A0  P1.0|--> LED
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

/*
 * Author: Nicholas Klein, Chris Anling
 * Created Date: 11/1/18
 * Last Edit: 11/26/18
 * Milestone 2
 */

#include <msp430.h>
#include <math.h>

float currTemp = 0.0;
float newTemp = 0.0;
unsigned int vin = 0;

void outputSetup(void)
{
    //output for PWM
    P1DIR |= BIT2;                              // Sets pin 1.2 to the output direction
    P1SEL |= BIT2;                              // BIT2 = TA0.1 output
    P1OUT &= ~BIT2;                             // Turn off

    //Led for UART
    P4DIR |= BIT7;                              // Sets pin 4.7 to the output direction
    P4OUT &= ~BIT7;                             // Turn off
}


void PWMSetup(void)
{
    TA0CTL = TASSEL_2 | MC_1 | TACLR;           // SMCLK set to UP mode, clear TAR
    TA0CCR0 = 255;                              // PWM Period

    TA0CCR1 = 0;                                // TA0 duty cycle is 0%
    TA0CCTL1 = OUTMOD_7;                        // Reset/Set
}


void UARTSetup(void)
{
    P4SEL |= BIT4 | BIT5;                       // BIT4 = TXD output || BIT5 = RXD input
    P3SEL |= BIT4 | BIT5;                       // BIT3 = TXD output || BIT5 = RXD input
    UCA1CTL1 |= UCSWRST;                        // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_1;                       // Sets USCI Clock Source to SMCLK
    UCA1BR0 = 3;                                // Setting the Baud Rate to be 9600
    UCA1BR1 = 0;                                // ^
    UCA1MCTL |= UCBRS_0 | UCBRF_3;
    UCA1CTL1 &= ~UCSWRST;                       // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt
}


void ADCSetup(void)
{
  ADC12CTL0 = ADC12SHT1_15 | ADC12SHT0_15 | ADC12MSC | ADC12ON | ADC12TOVIE | ADC12ENC | ADC12SC;
      // 1024 ADC12CLK cycles, first sample triggered, ADC12 on, conv-time overflow ie, enable conversion, start conversion
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12CTL2 = ADC12RES_2;                   // 12bit ADC12_A Resolution
  //ADC12MCTL0 =
  ADC12IE = ADC12IE0;                       // Enable interrupt
  ADC12IFG &= ~ADC12IFG0;                   // Clear interrupt flag
  P6SEL |= BIT0;                            // P6.0 ADC peripheral
  P6DIR &= ~BIT0;                           // P6.0 input
}


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                   // Stop Watchdog Timer
    outputSetup();
    PWMSetup();
    UARTSetup();
    ADCSetup();

    while (1)
    {
      ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion

      __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
      __no_operation();                       // For debugger
    }
}



#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    P4OUT |= BIT7; // Turn on the onboard LED

    switch(__even_in_range(UCA0IV,4))
      {
      case 0:
          break;                                // Vector 0 - no interrupt
      case 2:                                   // Vector 2 - RXIFG
        while (!(UCA0IFG & UCTXIFG));           // USCI_A0 TX buffer ready?
            UCA0IFG &= ~UCTXIFG;                // Clear the TX interrupt flag
            //need to convert currTemp to ASCII, what is currTemp as-is?
            UCA0TXBUF = currTemp;               //Transmit currTemp
            break;
      default:
          break;
      }

    if (UCA0IFG & UCRXIFG) {
        UCA0IFG &= ~UCRXIFG;                    // Clear the RX interrupt flag
        newTemp = UCA0RXBUF;                    // Read Data from UART
    }

    P4OUT &= ~BIT7; // Turn off the onboard LED
}

// Chris wanted me to swear in here so... butts & heck

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  6:                                      // Vector  6:  ADC12IFG0
      Nadc = ADC12MEM0;
      float currTemp = 27.3 / Nadc;  //obtains temperature in celcius based off of VR+ == 1.5
          // Nadc = (Vin/VR+)*4095 = (10E-3V/1C/1.5V)*4095
              
      float tempDiff = currTemp - newTemp;
      
      while(tempDiff != 0.0);
      {
          if (currTemp > newTemp)
          {
              if (TA0CCR1 < 255)
              {
                  TA0CCR1 += 5;                 //increments PWM cycle by constant
              }
          }
          else if (currTemp < newTemp)
          {
              if (TA0CCR1 < 255)
              {
                  TA0CCR1 -= 5;                 //decrements PWM cycle by constant
              }
          }
          tempDiff = currTemp - newTemp;        //reassigns tempDiff
      }
      UCA0TXBUF = currTemp;               //Transmit currTemp
  default: break;
  }
}
