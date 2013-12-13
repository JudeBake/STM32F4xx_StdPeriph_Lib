**STM32F4xx_StdPeriph_Lib**
===========================

Standard Peripheral libraries for STM32F4xx

Contains standard peripheral library from STM and library of linking layer
between STM library and FreeRTOS.

Librairies Contained
--------------------

1-  STM32F4xx_StdPeriph_Driver from STMicroelectronic.
2-  STM32F4xx_FreeRTOS_PeriphDriver.

1. STM32F4xx_StdPeriph_Driver
-----------------------------
This librairy is from ST Microelectronic and is delivered "AS-IS".

2. STM32F4xx_FreeRTOS_PeriphDriver
----------------------------------
This librairy contains second level driver for some peripherals of the STM32F4xx
uC. Those have been more or less extensivly tested and are as follow:

1- Asynchronous Serial Port for U(S)ART ports 1, 2, 3, 4, 5, 6.  
   Tested: Tests were made on initialization of port 3 for different setting.  
   Functional tests were made on port 3 in full duplex mode w/o hardware flow  
   control. Since the other port differ mostly on the initilization they were  
   assumed functionnal and w/o bugs.  

