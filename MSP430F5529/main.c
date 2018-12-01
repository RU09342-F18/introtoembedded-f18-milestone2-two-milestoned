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
 * Author: Nicholas Klein, Chris Anling, Scott Gordon
 * Created Date: 11/1/18
 * Last Edit: 11/28/18
 * Milestone 2
 */

#include <msp430.h>
#include <math.h>

float currTemp = 1.0;
int newTemp = 25;
int avgTemp = 1;
float tempDiff = 0.0;
unsigned int Nadc = 0;
int tempCounter = 0;
int testCounter = 0;
unsigned int temps[8] = {0, 0, 0, 0, 0, 0, 0, 0};


void PWMSetup(void)
{
    //output for PWM
    P1DIR |= BIT2;                              // Sets pin 1.2 to the output direction
    P1SEL |= BIT2;                              // BIT2 = TA0.1 output
    P1OUT &= ~BIT2;                             // Turn off

    TA0CTL = TASSEL_2 | MC_1 | TACLR;           // SMCLK set to UP mode, clear TAR
    TA0CCR0 = 1000;                             // PWM Period

    TA0CCR1 = 500;                              // TA0 duty cycle is 50%
    TA0CCTL1 = OUTMOD_7;                        // Reset/Set
}


void UARTSetup(void)
{
    P4SEL |= BIT4 | BIT5;                       // BIT4 = TXD output || BIT5 = RXD input
    UCA1CTL1 |= UCSWRST;                        // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_1;                       // Sets USCI Clock Source to SMCLK
    UCA1BR0 = 0x03;                             // Setting the Baud Rate to be 9600
    UCA1BR1 = 0x00;                             // ^
    //UCA1MCTL |= UCBRS_0 | UCBRF_3;
    UCA1MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                       // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt

    //Led for UART
    P4DIR |= BIT7;                              // Sets pin 4.7 to the output direction
    P4OUT &= ~BIT7;                             // Turn off
}


void ADCSetup(void)
{
    /*
  ADC12CTL0 = ADC12SHT1_15 | ADC12SHT0_15 | ADC12MSC | ADC12ON | ADC12TOVIE | ADC12ENC | ADC12SC;
      // 1024 ADC12CLK cycles, first sample triggered, ADC12 on, conv-time overflow ie, enable conversion, start conversion
  ADC12CTL1 = ADC12SHP;                         // Use sampling timer
  ADC12CTL2 = ADC12RES_2;                       // 12bit ADC12_A Resolution
  //ADC12MCTL0 =
  ADC12IE = ADC12IE0;                           // Enable interrupt
  ADC12IFG &= ~ADC12IFG0;                       // Clear interrupt flag
  P6SEL |= BIT0;                                // P6.0 ADC peripheral
  P6DIR &= ~BIT0;                               // P6.0 input
  */

    //timer for adc capture
    TA1CTL = TASSEL_2 | MC_1 | TACLR | TAIE;    // SMCLK set to UP mode, clear TAR, enable interrupt
    TA1CCR0 = 32;                               // PWM Period, 1ms

    ADC12CTL0 = ADC12SHT02 | ADC12REFON | ADC12ON;           // Sampling time, 1.5v ref on, ADC12 on
    ADC12CTL1 = ADC12SHP;                       // Use sampling timer
    ADC12IE = 0x01;                             // Enable interrupt
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                              // P6.0 ADC option select
    P6DIR &= ~BIT0;                             // P6.0 input
    P1DIR |= 0x01;                              // P1.0 output
}


int readTemp(float Vin)
{
    int Vout;
    if (tempCounter == 0) {
        temps[tempCounter] = Vin;
        tempCounter++;
    }
    else if (tempCounter < 7) {
        temps[tempCounter] = temps[tempCounter - 1];
        tempCounter++;
    }
    else {
        temps[tempCounter] = temps[tempCounter - 1];
        tempCounter = 0;
    }
    Vout = (temps[0] + temps[1] + temps[2] + temps[3] + temps[4] + temps[5] + temps[6] + temps[7])/8;

    return Vout;
}


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                   // Stop Watchdog Timer
    PWMSetup();
    UARTSetup();
    ADCSetup();

    __bis_SR_register(GIE);                     // LPM0, ADC12_ISR will force exit
    while (1)
    {
      __no_operation();                         // For debugger
    }
}


#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    P4OUT |= BIT7; // Turn on the onboard LED

    switch(__even_in_range(UCA1IV,4))
      {
      case 0:
          break;                                // Vector 0 - no interrupt
      case 2:                                   // Vector 2 - RXIFG
        while (!(UCA1IFG & UCTXIFG));           // USCI_A0 TX buffer ready?
        UCA1IFG &= ~UCTXIFG;                    // Clear the TX interrupt flag
        UCA1IFG &= ~UCRXIFG;                    // Clear the RX interrupt flag
        //need to convert currTemp to ASCII, what is currTemp as-is?
        UCA1TXBUF = (int)currTemp & 0x0FF;      //Transmit currTemp
        newTemp = UCA1RXBUF;                    // Read Data from UART

        break;
      default:
          break;
      }

    P4OUT &= ~BIT7;                             // Turn off the onboard LED
}

// Chris wanted me to swear in here so... butts & heck


#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)

{
    switch(__even_in_range(TA1IV,14))
      {
        case 14:                                 // overflow
            ADC12CTL0 |= ADC12SC;                // Start sampling/conversion
            TA1CTL &= ~TAIFG;                    // disable interrupt flag
            break;
        default: break;
      }
}


#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV,34))
    {
    case  6:                                      // Vector  6:  ADC12IFG0
      Nadc = ADC12MEM0;
      currTemp = ((((Nadc/4095.)*2.75) - 0.5) * 100) + 10;
              //(((Nadc/4095.)*5)/0.0375) - 5;
          //currTemp = (((Nadc/4095.)*1.5) - 0.5) * 100;             //obtains temperature in celcius based off of VR+ == 1.5
          //getting close values to expected but they're negative
          // Nadc = (Vin/VR+)*4095 = (((10E-3V/1C)-0.5V)/1.5V)*4095
          // old eq: 27.3 / (Nadc - 1365)
          // old eq2: (((Nadc/4095.)*1.5) - 0.5) * 100,redone eq by Russel and Nick
          //eq other team used: ((((3.3)/4096)-0.424)*160)

      //readtemp();                       //for debug -- causes error, abandoned

      //avgTemp = readTemp(currTemp);

      tempDiff = currTemp - newTemp;

      if (tempDiff >= 0.1 || tempDiff <= -0.1)
      {
          if (currTemp > newTemp)
          {
              if (TA0CCR1 < 1000)
              {
                  TA0CCR1 ++;                 //increments PWM cycle
              }
          }
          else if (currTemp < newTemp)
          {
              if (TA0CCR1 > 20)
              {
                  TA0CCR1 --;                 //decrements PWM cycle
              }
          }
      }
      while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
      UCA1IFG &= ~UCTXIFG;                      // Clear the TX interrupt flag
      UCA1TXBUF = (int)currTemp & 0x0FF;        //Transmit currTemp
      break;
    default: break;
    }
    if (testCounter >= 1056) { //for testing
      testCounter = 0;
    }
    else {
      testCounter++;
    } //end test section
    }
