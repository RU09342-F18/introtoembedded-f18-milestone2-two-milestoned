# Milestone 2: Closed Loop Systems
A closed loop system regulates itself based on feedback. This example uses a TI MSP-EXP430F5529LP microprocessor to regulate temperature of a TMP36 Low Voltage Temperature Sensor being heated by a LM809AC 5-volt regulator using a computer fan. This is done by sending a temperature in centigrade over UART using a serial terminal program such as RealTerm. The microcontroller obtains the current temperature by interpreting the voltage sent from the Temperature Sensor. The desired temperature is compared against the current temperature, the resultant comparison sends a signal to increase or decrease the duty cycle of the pulse width modulation if there is a disparity to control the speed of the cooling fan.

## Analog to Digital Converter (ADC)
The ADC functions by comparing an input voltage to a reference voltage and stores a value equal to the ratio of the two relative to the bit range available in that ADC, in this case a 12bit ADC is used, therefore the number stored is from 0 to 4095. This value is used to calculate the current temperature of the temperature sensor.

## Pulse Width Modulation (PWM)
PWM is a power control where current is regulated by only allowing it to flow some set percentage of time called the Duty Cycle. This is done by setting one register as the peak value, which sets the period as the number of assigned clock cycles, and setting another register as the number of cycles that the current may flow before being stopped for the remainder of the clock cycles in the period. This system increments or decrements the duty cycle register by 1 out of a maximum of 1000 each milisecond based on the comparison of the current and desired temperatures.

## Universal Asynchronous Reciever Transmitter (UART)
UART is a serial communication method that transmits and recieves data without any need for a clock. Here, UART communicates with a computer using a serial terminal program where the desired temperature is sent from the terminal and taken in through the RX buffer so it can be used to compare against the current temperature. Simultaneously the TX buffer stores the current temperature which is sent back to the terminal program every milisecond to be displayed for the user.

## Circuit Design
<img src="https://github.com/RU09342-F18/introtoembedded-f18-milestone2-two-milestoned/blob/master/Milestone2Circuit.JPG" height="500" width="500">
