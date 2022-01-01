# DistanceSensorSecuured_DCmotorController
Basic hobby project: Distance Auto-Stop secured buzzer and led alerted two direction brushed DC motor controller with speed control

   Created on: Dec 13, 2021
       Author: Suleyman Oflaz
   Used Board: STM32F407G-DISC1

Distance Auto-Stop secured buzzer and led alerted two-direction brushed DC motor controller with speed control.
Motor-start controlled by a switch and also there is a motor-on status led.
Motor direction control was also implemented. Direction can be changed by a push button. There are 2 motor-direction status LEDs.
Motor speed can be controlled over a potentiometer.
The system has a distance sensor that gets the system into the emergency status and causes emergency motor stop when reading a close range.
The motor switch must switch off and on after that to restart when the motor stops in emergency status.
There are buzzer and LEDs that depends on the distance read from the sensor. Sensor-led blinks when detecting a potential object. 
If the object is closer than the emergency distance emergency led and buzzer alert turns on. The system is put into an emergency state and waits for a restart sequence.

STM32F407-DISC1 board,
HC-SR04 as a distance sensor,
L298N as a brushed DC motor driver,
were used.
  
Used Pins:
		PWM Signal Outputs: -- Brushed DC Motor Drive			
         - PB7
 			- PB8
 		Square Signal Output: -- Buzzer Driven
 			- PA2
 		Sensor I/Os: -- HC-SR04
 			- PA1 -- Output (Trigger)
 			- PA3 -- Input (Echo)
 		ADC Input:
 			- PB0 (10k Pot)
 		LED Outputs:
 			- PB11 -- Sensor Light
 			- PB12 -- Emergency Light
 			- PB13 -- Motor Direction Light 1
 			- PB14 -- Motor Direction Light 2
 			- PB5  -- Motor Status Light
 		UART I/Os:
 			- PD8 -- TX
 			- PD9 -- RX
 		Button/Switch Inputs:
 			- PA0  -- Motor Direction Change Button
 			- PB15 -- Motor-On Switch (Active Low)
