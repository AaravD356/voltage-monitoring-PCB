### UMD Mars Rover Electrical Team

### Voltage Monitoring PCB
This project involves a custom PCB designed to monitor the voltage of a multi-cell battery pack and transmit the readings over a CAN bus. The core of the system is an STM32 microcontroller, which reads the voltage of each cell through voltage divider circuits. The measured values are then transmitted via a SN65HVD230 CAN transceiver to a Jetson Nano, which acts as the receiver and processor.

### System Overview
The PCB includes six voltage divider circuits, one for each battery cell, to scale down the cell voltages to a level readable by the STM32's ADC inputs. The STM32 continuously reads these voltages and formats the data into a CAN message. The SN65HVD230 transceiver handles communication over the CAN bus, ensuring reliable data transfer to the Jetson Nano or any compatible receiver.

### CAN Communication
Each CAN message contains the measured voltages, formatted as an array of 16-bit unsigned integers representing millivolts. A static CAN ID is used to identify messages from the monitoring board. This ID can be customized in the STM32 firmware if needed.
