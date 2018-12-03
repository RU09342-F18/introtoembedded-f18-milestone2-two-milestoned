# Milestone 2: Closed Loop Systems
A closed loop system regulates itself based on feedback. This example uses a TI MSP-EXP430F5529LP microprocessor to regulate temperature of a TMP36 Low Voltage Temperature Sensor being heated by a LM809AC 5-volt regulator using a computer fan. This is done by sending a temperature in centigrade over UART using a serial terminal program such as RealTerm. The microcontroller obtains the current temperature by interpreting the voltage sent from the Temperature Sensor. The desired temperature is compared against the current temperature, the resultant comparison sends a signal to increase or decrease the duty cycle of the pulse width modulation if there is a disparity to control the speed of the cooling fan.

## Analog to Digital Converter (ADC)
The ADC functions by comparing an input voltage to a reference voltage and stores a value equal to the ratio of the two relative to the bit range available in that ADC, in this case a 12bit ADC is used, therefore the number stored is from 0 to 4095. This value is used to calculate the current temperature of the temperature sensor.

## Pulse Width Modulation (PWM)
PWM is a power control where current is regulated by only allowing it to flow some set percentage of time called the Duty Cycle. This is done by setting one register as the peak value, which sets the period as the number of assigned clock cycles, and setting another register as the number of cycles that the current may flow before being stopped for the remainder of the clock cycles in the period. This system increments or decrements the duty cycle register by 1 out of a maximum of 1000 each milisecond based on the comparison of the current and desired temperatures.

## Universal Asynchronous Reciever Transmitter (UART)
UART is a serial communication method that transmits and recieves data without any need for a clock. Here, UART communicates with a computer using a serial terminal program where the desired temperature is sent from the terminal and taken in through the RX buffer so it can be used to compare against the current temperature. Simultaneously the TX buffer stores the current temperature which is sent back to the terminal program every milisecond to be displayed for the user.

## Circuit Design
The goal of the circuit is to facilitate the heating and cooling of the temperature sensor. It can be broken down into three separate circuits, the fan circuit, the 5V regulator heating circuit, and the sensor circuit.

### Fan Circuit
A CPU cooling fan is connected to a 12V VCC. 12V is chosen as the fan needs 12V to run. A flyback zener diode in parallel controls power spikes. They are connected to an NMOS drain. This NMOS serves as a low-side switch which controls the PWM of the fan. The source of the NMOS is connected to ground and the gate connects to two resistsors. The first goes to Pin1.2 of the microcontroller where the signal for PWM origionates. The second is a very large pull-down resistor that goes to ground so the gate is off by default. A paper cone is taped to the fan to better focus the air movement which is pointed directly onto the temperature sensor.

### Heating Circuit
A 5V regulator is connected to the 12V VCC on the Vin pin. 12V are used since the power supply already has that supplied and the voltage needs to be more than the 5V that will be regulated to create heat. The Vout Pin is connected to an 8W power resistor which is then connected to ground. This pulls more current so the regulator can heat up. The ground pin of the regulator connects directly to ground. The regulator must be placed so it can touch the temperature sensor.

### Sensor Circuit
The Temperature sensor's Vin pin is connected to a 5V VCC as the TMP36 sensor used only operates from 2.7V to 5.5V. The ground pin is connected directly to ground, and the Vout Pin is connected to the negative terminal on an op-amp which is used as a buffer. The buffer opamp serves to reduce noise by stopping extra voltage from being pulled. The output of the Op-Amp is connected back to the positive terminal and to a 850Ω resistor. That resistor is connected to a 15uF capacitor and Pin6.0 which is the input of the ADC. The resistor capacitor combo is a low-pass filter to reduce noise. The Sensor has to be touching the 5V regulator to heat up.

### Circuit Diagram
<img src="https://github.com/RU09342-F18/introtoembedded-f18-milestone2-two-milestoned/blob/master/Milestone2CircuitV2.JPG" height="500" width="500">

## Parts List
• MSP-EXP430F5529LP Microcontroller
• Solderless Breadboard
• TMP36 Temperature Sensor
• LM7809AC 5V Regulator
• Foxconn AFB0712HHB 435063-001 CPU Fan
• 2N7000G MOSFET
• NE5532P Op-Amp
• RIE 1310 8W power resistor
• 150Ω Resistor
• 1MΩ Resistor
• 850Ω Resistor
• 15uF Capacitor
• Zener Diode
