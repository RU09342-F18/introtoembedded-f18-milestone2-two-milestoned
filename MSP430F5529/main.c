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
 * Last Edit: 11/30/18
 * Milestone 2
 */

#include <msp430.h>
#include <math.h>

// declare variables
float currTemp = 1.0;                                           // Defines current temperature interpreted by ADC
int newTemp = 25;                                               // Defines desired temperature given by UART
float tempDiff = 0.0;                                           // Difference between current temperature and desired temperature
unsigned int Nadc = 0;                                          // interpreted input from ADC12MEM0 register of ADC


void PWMSetup(void) {
    // Output for PWM
    P1DIR |= BIT2;                                              // Sets pin 1.2 to the output direction
    P1SEL |= BIT2;                                              // BIT2 = TA0.1 output
    P1OUT &= ~BIT2;                                             // Sets pin 1.2 to "off" state

    // Clock for PWM
    TA0CTL = TASSEL_2 | MC_1 | TACLR;                           // SMCLK (32kHz) set to UP mode, clear TAR
    TA0CCR0 = 1000;                                             // PWM Period = 1000 clock cycles
    TA0CCR1 = 500;                                              // TA0 duty cycle is 50%
    TA0CCTL1 = OUTMOD_7;                                        // Reset/Set mode
}


void UARTSetup(void) {
    P4SEL |= BIT4 | BIT5;                                       // Pin4.4 set as TXD output,  Pin4.5 set as RXD input
    UCA1CTL1 |= UCSWRST;                                        // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_1;                                       // Sets USCI Clock Source to SMCLK (32kHz)
    UCA1BR0 = 0x03;                                             // Setting the Baud Rate to be 9600
    UCA1BR1 = 0x00;                                             // Setting the Baud Rate to be 9600
    UCA1MCTL = UCBRS_3+UCBRF_0;                                 // Modulation UCBRSx=3, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                                       // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                                           // Enable USCI_A0 RX interrupt

    // Led for UART - Lights up when UART is in use
    P4DIR |= BIT7;                                              // Sets pin 4.7 to the output direction
    P4OUT &= ~BIT7;                                             // Sets pin 4.7 to "off" state
}


void ADCSetup(void) {
    //timer for adc capture
    TA1CTL = TASSEL_2 | MC_1 | TACLR | TAIE;                    // SMCLK (32kHz) set to UP mode, clear TAR, enable interrupt
    TA1CCR0 = 32;                                               // PWM Period, 1ms

    ADC12CTL0 = ADC12SHT02 | ADC12REFON | ADC12ON | ADC12ENC;   // Sample and hold time set to 16 clock cycles, 1.5v reference voltage on, ADC12 on, Enable ADC12 Conversion
    ADC12CTL1 = ADC12SHP;                                       // Sampling signal sourced from sampling timer
    ADC12IE = 0x01;                                             // Enable ADC12 interrupt

    // ADC12 Output
    P6SEL |= 0x01;                                              // P6.0 ADC option select
    P6DIR &= ~BIT0;                                             // Sets pin 6.0 to the input direction
}


void main(void) {
    // Run set up functions
    WDTCTL = WDTPW | WDTHOLD;                                   // Stop Watchdog Timer
    PWMSetup();
    UARTSetup();
    ADCSetup();

    __bis_SR_register(GIE);                                     // Enables Global Interrupt
    while (1)
    {
      __no_operation();                                         // For debugger - Informs debugger it will receive no input
    }
}


// UART Interrupt Vector
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    P4OUT |= BIT7;                                              // Turn on the UART onboard LED

    switch(__even_in_range(UCA1IV,4)) {                         // Checks which UART interrupt vector has been triggered
        case 0:                                                 // Vector 0 - no interrupt
            break;
        case 2:                                                 // Vector 2 - RXIFG
            while (!(UCA1IFG & UCTXIFG));                       // Checks if USCI_A0 TX buffer is ready to recieve input
            UCA1IFG &= ~UCTXIFG;                                // Clear the TX interrupt flag
            UCA1IFG &= ~UCRXIFG;                                // Clear the RX interrupt flag
            UCA1TXBUF = (int)currTemp & 0x0FF;                  // Transmit currTemp through UART
            newTemp = UCA1RXBUF;                                // Read received Data from UART

            break;
        default: break;
    }

    P4OUT &= ~BIT7;                                             // Turn off the UART onboard LED
}


//Timer_A Interrupt Vector- used to regulate readings from ADC12
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void) {
    switch(__even_in_range(TA1IV,14)) {                         // Checks which Timer_A interrupt vector has been triggered
        case 14:                                                // Timer_A overflow vector
            ADC12CTL0 |= ADC12SC;                               // Start ADC12 sampling/conversion
            TA1CTL &= ~TAIFG;                                   // disable Timer_A interrupt flag
            break;
        default: break;
    }
}


// ADC12 Interrupt Vector
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    switch(__even_in_range(ADC12IV,34)) {                       // Checks which ADC12 interrupt vector has been triggered
    case  6:                                                    // Vector  6:  ADC12IFG0
        Nadc = ADC12MEM0;                                       // Obtains raw reading from ADC12MEM0 register
        currTemp = ((((Nadc/4095.)*2.75) - 0.5) * 100) + 10;    // Obtains temperature in celcius based off of ADC12 reading and reference voltage
                                                                // This equation is edited from the derived calculation to create more accurate values
                                                                // Derived calculation: ((((Nadc/4095.)*[referenceVoltage]) - 0.5) * 100)

        tempDiff = currTemp - newTemp;                          // Calculates temperature difference between current temperature and desired temperature

        if (tempDiff >= 0.1 || tempDiff <= -0.1) {              // Determines if temperature difference is within an acceptable range
            if (currTemp > newTemp) {                           // Cools PTAT sensor if the system detects the current temperature as higher than the desired temperature
                if (TA0CCR1 < 1000) {                           // Ensures that the PWM cycle is never set above 100%
                    TA0CCR1 ++;                                 // Increments PWM cycle
                }
          }
          else if (currTemp < newTemp) {                        // Allows 5-volt regulator to heat PTAT sensor if the system detects the current temperature as lower than the desired temperature
              if (TA0CCR1 > 20) {                               // Ensures that the PWM cycle is never set below 2% - ensuring the fan never shuts off completely
                  TA0CCR1 --;                                   // Decrements PWM cycle
              }
          }
        }
        while (!(UCA0IFG&UCTXIFG));                             // Checks if USCI_A0 TX buffer is ready to recieve input
        UCA1IFG &= ~UCTXIFG;                                    // Clear the TX interrupt flag
        UCA1TXBUF = (int)currTemp & 0x0FF;                      // Transmit currTemp through UART
        break;
    default: break;
    }
}
