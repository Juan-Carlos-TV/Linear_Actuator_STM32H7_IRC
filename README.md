# Linear_Actuator_STM32H7_IRC
## Project

<div align="center"><img src="https://m.media-amazon.com/images/I/81Twh05MQFL._SL1500_.jpg" width="auto" height="400" /></div>

This project consist in the creation of an **smart linear actuator** based on <a href="https://www.st.com/en/evaluation-tools/nucleo-h745zi-q.html">Nucleo STM32H745ZI-Q</a>  microcontroller board as part of *Design of Avanced Embebed Systems* subject in Tecnológico de Monterrey, in colaboration with John Deere.

The main idea of this smart system is the internet conection through Esp32 in order to send and receive messages from an Arduio IoT Cloud websocket application which will be transmited via CAN Bus comunnication to the STM32 Nucleo Board which will be running the tasks using a real time OS using FreeRTOS supported by the STM32 Cube IDE. The microcontroller will execute:
<ol>
  <li>Motor driving control task in order to elongate the cylinder to the desired postion.</li>
  <li>Diagnosis task to detect overcurrent, undercurrent, overvoltage and undervoltage (in this case, we are simulating the entries and the threshold only for demostrative purposes). </li>
  <li>Can Communication rasks to read the desired position and send actual position and status of the system./li>
</ol>

### Block Diagram
<img src="https://i.imgur.com/pwVpeBz.png" width="auto" height="400" />

In this diagram there are shown 3 main components of this project: 
<ul>
  <li>The microcontroller (STM32 Nucleo H745ZI-Q Board)
  <li>The actuator
  <li>The ESP32
</ul>
In this document we will concentrate the most in the actuator and microcontroller part.
Once we have defined the tasks of the microcontroller, there are some special considerations about the motor driving:
<ul>
  <li>Since at the momment of starting the program we are not able to know the last position of the actuator, we need to make an special function for calibration propurses.
  <li>The feedback could be provided by several different components: rotary, optical or half effect encoders, distance sensors or even, if the longest position possible is not to long, a linear potentiometer. We will describe our choice later.
</ul>

### Components
These are the main components we used for our implementation:
<ul>
  <li> MCP2515 CAN Bus Module: one connected via SPI to the ESP32 while other was modified to only use the CAN Transciver since we are using the inbuild FDCAN Master of STM32 (more explanation on the scheme section).
</ul>    
    
<img src="https://media.naylampmechatronics.com/1053-superlarge_default/modulo-can-mcp2515.jpg" width="auto" height="250" />
    
**NOTE:** We made this modification because the team were not able to get a common CAN Transciver in time, but any of them should work. One of the most common is the MCP2551 which has its own module.

<img src="https://electronilab.co/wp-content/uploads/2021/06/Modulo-CAN-Bus-MCP2551-1.jpg" width="auto" height="250" />

<ul>   
  <li>Gearmotor: for our implementaion we used a  <a href="https://www.andymark.com/products/hex-pg-series-gearboxes-options">PG27 Gearmotor</a> which is made from RS775 motor and PG71 Gearbox from Andymark. This gearmotor also has an inbuild Hal effect encoder of 7 ppr. This commponent needs and input voltage of 12V.
</ul>
<img src="https://andymark-weblinc.netdna-ssl.com/product_images/hex-pg-series-gearboxes-options/5bd8c03a61a10d5948a536e9/detail.jpg?c=1540931642" width="auto" height="250" />

**NOTE:** There are several other options for gear motors and encoders, as the common yellow gearmotor used in the classic robot proyects, pololu gearmotors, stepper motors, a common dc motor with your own gear box. Some of this models ara available with inbuild encoders (commonly hal effect) and these are the ones we recommend. 
Deppending on the precision you need, you'll look for more or less pulses per revolution (ppr). Only consider that it's counting the revolution of the motor, so the total ppr of the gearmotor after the reduction will be *the ppr of the encoder multiplied by the reduction of the gearbox*

<img src="https://hetpro-store.com/images/detailed/17/encoder-02.jpg" width="auto" height="200" /> 

**EXAMPLE:** *if I have a 7 ppr encoder and I have a reduction of 50:1 then the exact number of pulses for one gear motor revolution is:*

*ppr × reduction = 7 ppr × 50 = 350 ppr*

In our case, 7 ppr were enough to get a total number of 497 pulses. Very presice for our implementation.

<ul>
  <li>External power supply of 12V and 5 A. For this application was enough
  <li>H-Bridge BTS7960: high power motor driver needed to manage the high voltage and current of our external supply
</ul>

<img src="https://resources.claroshop.com/medios-plazavip/mkt/6026f95cd502c_modulo-puente-h-bts7960-43a-arduino-pic3jpg.jpg?scale=340" width="auto" height="250" />

<ul>
  <li>Common limit switch for calibration propourses: at the initialization we will rotate backwards until the switch goes high and then we are going to rotate forward until initial position.
  <li>Two potentiometers in order to simulate the output voltage of current and voltage sensors
    
</ul>

## Scheme
## Considerations!
