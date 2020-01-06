# FreeRTOSBinaryClock
FreeRTOS running a Binary Clock using the MAX7219 8x8 LED Matrix as the Display. The Microcontroller is the ATMega 328P
FreeRTOS Binary Clock on the ATmega328P Arduino Platform
ED6422 – Real Time System Design 
Graham Claffey*, Student Member
18296661
University of Limerick, Co. Limerick, Ireland
Email: 18296661@student.ul.ie
 
 
 
Abstract – The goal of this project is to create a binary clock on the ATmega328P Arduino platform, using an RTOS (Real-Time Operating System) to manage all the functionality under the hood. The RTOS that was chosen for this project is FreeRTOS. We will use this RTOS to send our binary clock calculations to an 8x8 LED Matrix to display the time in a binary format. The LED Matrix chosen for this project is the Max7219. The software implementation for this pro-ject will be using Atmel built-in Instruction Set for their chips known as AVR. The AVR instruction set is usual-ly coded in C and is commonly referred to as AVR-C.
Keywords – Atmega328P, Arduino, AVR, Binary Clock, C, FreeRTOS, Max7219   

I.	INTRODUCTION
A binary clock is as the name suggests a clock that displays its time in a binary format, either 1 or 0, on or off etc. In this case the display will be on the LED matrix, we determine the time by viewing which state the LED is in. Each row of the 8x8 LED matrix will correspond to a certain time value on a regular base10 clock, for in-stance the first row will be our seconds, second row will be minutes, third will be hours, forth will be days etc. So the rows will account for our time values and each of the columns on the 8x8 LED matrix will have a weighted value repre-senting the number of states the LEDs of that row can be lit before that column lights up. To paint a clearer picture, starting from the right more LED of any row its weighted value is 1 be-cause there is only one state it can be in before it lights up. These weighted column values from right to left is represented in binary as 2 to the power of column number, the right most column being 1 and the left most being column 7.
The other component of this project is FreeRTOS running on the ATmega328P Arduino platform. The ATmega328P we are using is implemented on an Arduino UNO board, this will be were the FreeRTOS Kernel will be installed and use as a so called “base of operation” or physical hub from where we can connect our physical devic-es. The Adruino will be connected via USB cable to a PC/Laptop which acts as the source of power for the Arduino and all the devices connected to it. The USB will also act as our method to com-municate between the PC and our physical me-dia. Our Max7219 will also be connect to the Arduino. 

II.	HARDWARE 
A more in-depth look at the hardware used in this project. Here we will see the inner-workings of the Arduino and the Max7219 to get a better understanding of the capabilities and limitations of both of these components. 
Max7219
The Max7219 we have been referring to is actu-ally named because of the chip used in this com-ponent, which is the Max7219CGN from MAXIM. The physical make-up of this compo-nent involves a PCB (printed circuit board), the Max7219 chip, an 8x8 LED matrix, 5 output (right in Fig. 1.) and 5 input pins (left in Fig. 1.).
Since the output pins are not used we will look at the input pins.
Fig. 1.	The Max7219 physical device
The 5 input pins we use in this project are: [1] 
1.	VCC – Positive supply voltage. Connect to +5V.
2.	GND – Ground.
3.	DIN – Serial-Data Input. Data is loaded into the internal 16-bit shift register on CLK’s rising edge.
4.	CS – Chip-Select Input. Serial data is loaded into the shift register while CS is low. The last 16 bits of serial data are latched on CS’s rising edge.
5.	CLK – Serial-Clock Input. Maximum rate is 10MHz. On CLK’s rising edge, data is shifted into the internal shift reg-ister. On CLK’s falling edge, data is clocked out of DOUT.
Now that we have an understanding of the pins used we can take a look at the inner workings using a functional diagram of the Max7219.
Fig. 2.	Functional diagram of the Max7219
ATmega328P Microcontroller
The ATmega328P Microcontroller used in this project is a SMD (Surface Mounted Device) package. Mounted directly to the Arduino UNO board were the 32 pins are connected to the var-ious peripherals on the board. It is an 8-bit AVR RISC-based microcontroller that runs at a fre-quency of 16MHz. A more detailed breakdown of the specifications [2]:
•	Program Memory Type:		Flash
•	Program Memory Size (KB)	32
•	CPU Speed (MIPS/DMIPS)	20
•	SRAM (KB)			2,048
•	Data EEPROM/HEF (bytes)	1024
•	Digital Communication Peripherals        1-UART, 2-SPI, 1-I2C
•	Capture/Compare/PWM Peripherals           1 Input Capture, 1 CCP, 6 PWM.
•	Timers 		        2 x 8-bit, 1 x 16-bit
•	Number of Comparators		1
•	Temperature Range (°C)	           -40 to 85
•	Pin Count			32
•	Low Power			Yes
•	Operation Voltages	           1.8-5.5V
•	A/D converter			10-bit
The device also has a programmable watchdog timer with an internal oscillator, and five soft-ware sel  ectable power saving modes. 
Fig. 3.	ATmega328P pinout diagram
III.	FREERTOS
FreeRTOS is a real-time operating system kernel for embedded systems. FreeRTOS also acts like an API allowing users to implement and custom-ise their very own projects using FreeRTOS. Fre-eRTOS is free open-source software distributed under the MIT Licence. FreeRTOS is designed to be small consisting of only a few files, mainly written in C, but there are a few assembly in-structions included.
The Setup of FreeRTOS for this project came with some considerations such as, FreeRTOS is not directly setup to be compatible with the AT-mega328P, but there is an officially supported microcontroller that is very similar to the AT-mega328P. This is the official ATmega323 port, so what we needed to do was to customise this port to suit our needs with the ATmega328P. We downloaded the FreeRTOS kernel and then stripped out all the unnecessary files that were either for specific microcontrollers or implemen-tation of features that we did not require. 
File changes
Two files that will be talked about due to the changes that were made for this project are:
•	port.c
•	FreeRTOSConfig.h
port.c – The changes that were made in this file were related to timers and output compares.
Fig. 4.	portCOMPARE_MATCH_A_INTERRUPT_ENABLE
Also one of the Timers need to be changed for our needs.
Fig. 5. TIMSK changed to TIMSK1 
Finally for port.c the timer1 compare vector was changed from signal output compare.
Fig. 6. Timer 1 Compare Vector changed
FreeRTOSConfig.h – As for the configuration file for FreeRTOS, we added/switch on features are they were needed in relation to the project. The FreeRTOSConfig.h file comes with a host of set-tings to tweak. Here is what the final product of that tweaking looked like after the project was finished[3].
Fig. 7. FreeRTOSConfig.h setup at the end of the project
API Features Used
In this project we used a variety of API features to help implement the creation, running and dis-play of the binary clock. Some examples of the features used[4][5][6][7]:
•	ISR() – Interrupt Service Routine.
•	SemaphoreGiveFromISR() – to give the Semaphore Handler.
•	SemaphoreTake() – to take the given semaphore.
•	xSemaphoreCreateBinary() – to create a binary semaphore.
•	xQueueCreate() – to create a queue.
•	xQueueSend() – to send data to the queue.
•	xQueueRecieve() – to receive data from the queue.
•	xTaskCreate() – to create our tasks.
•	vTaskStatScheduler() – to start the task scheduler.
•	vTaskDelay() – to delay the task for a specific amount of time.
•	QueueHandle_t – setup a queue handler
•	xSemaphoreHandle – setup a semaphore handler.

IV.	SOFTWARE DESIGN IMPLEMENTATION

Overview of the Software design behind the bi-nary clock project. 
We start with the 16MHz clock (F_CPU the ATmage328P default clock speed). Using Timer0 which is an 8-bit timer, we use prescaling and overflow to achieve a frequency of 1Hz. With Timer0 set at 1Hz we can use this to do an output compare vector using the ISR. This causes the ISR to trigger every second. When the ISR trig-gers we send a binary semaphore from the ISR to Task1.
Task1 is responsible for clock calculations, were it sends the results of these clock calculations a queue via xQueueSend().
Task2 is responsible for the LED Matrix Driver, it waits until there is some data in the queue, where then it will receive that data via the xQueueReceive() API call. Using the data from the queue, Task2 can format the data to be out-put to the Max7219.
Task3 is responsible for the UART Drivers, us-ing the values of the global variables which are being updated every second, to first convert the data into a char of a base10 representation using the stdlib.h function itoa() and then copy this data into a buffer to be send via the UART.
Task4 is responsible for Monitor Task, this is the status LEDs located at the last to rows, row 6 and 7. 


V.	CONCLUSION

In conclusion this has been an ex-tremely informative project in relation to learning about FreeRTOS and pro-gramming on the ATmega328P using the AVR ISA. As an introductory pro-ject to RTOS design and development it managed to succeed in not only bol-stering my interest in seeing what kind of other projects that I can create, but also in improving my skills within a multi-task environment. I’m already looking forward to my next project which would be to implement multiple Max7219 on an ARM Cortex chip and creating a binary, digital base10 and Temperature display in which the user can switch between the different modes of operation. The value of this project turned out to be very success-ful.  
