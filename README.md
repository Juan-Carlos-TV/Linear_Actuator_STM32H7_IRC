# Linear_Actuator_STM32H7_IRC
## Project

<div align="center"><img src="https://m.media-amazon.com/images/I/81Twh05MQFL._SL1500_.jpg" width="auto" height="400" /></div>

This project consist in the creation of an **smart linear actuator** based on <a href="https://www.st.com/en/evaluation-tools/nucleo-h745zi-q.html">Nucleo STM32H745ZI-Q</a>  microcontroller board as part of *Design of Avanced Embebed Systems* subject in Tecnológico de Monterrey, in colaboration with John Deere.

The main idea of this smart system is the internet conection through Esp32 in order to send and receive messages from an Arduio IoT Cloud websocket application which will be transmited via CAN Bus comunnication to the STM32 Nucleo Board. The microcontroller will execute:
<ol>
  <li>Motor driving control task in order to elongate the cylinder to the desired postion.</li>
  <li>Diagnosis task to detect overcurrent, undercurrent, overvoltage and undervoltage (in this case, we are simulating the entries and the threshold only for demostrative purposes). </li>
  <li>Can Communication rasks to read the desired position and send actual position and status of the system./li>
</ol>

### Block Diagram
### Components

## Scheme
## Considerations!
