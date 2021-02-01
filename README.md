# CYW920819EVB-02

### Overview

The Cypress CYW920819EVB-02 Evaluation Kit enables you to evaluate and develop single-chip Bluetooth applications using the CYW20819, an ultra-low-power dual-mode Bluetooth 5.2 wireless MCU device. The CYW20819 is a stand-alone baseband processor with an integrated 2.4 GHz transceiver supporting Bluetooth LE/BR/EDR. Manufactured using advanced CMOS low-power process, the CYW20819 employs high integration to reduce external components, thereby minimizing the device’s footprint and cost. This kit helps evaluate device functionality and develop applications quickly for faster time-to-market. CYW20819 supports Bluetooth SIG-certified Mesh v1.0. CYW20819EVB-02 can also be used to develop Bluetooth LE Mesh applications on CYW20819 device using ModusToolbox.

* 62-FBGA CYW20819 dual-mode (LE/BR/EDR) Bluetooth 5.2-compliant wireless MCU
* Arduino compatible headers for hardware expansion
* On-board sensors - a 9-axis motion sensor (3D digital linear acceleration sensor, 3D digital angular rate sensor, and 3D digital magnetic sensor) and a thermistor
* User switch and LEDs
* USB connector for power, programming and USB-UART bridge

### Kit Contents

CYW920819EVB-02 Evaluation board
USB Standard-A to Micro-B cable
Quick Start Guide


### Power Estimator Library
Power Estimator library provides an estimate of power consumed by a CYW920819EVB-02 device. Follow the below mentioned steps to get power estimation of the device:

Step 1: Enable Power Estimator Library add the following in application makefile:

```
	POWER\_ESTIMATOR=1
	CY\_APP\_DEFINES += -DWICED\_POWER\_LOGGER\_ENABLE
	COMPONENTS += wpl\_lib
```
Step 2: Launch Power Estimator Tool from ModusToolbox 2.0 IDE on the host machine:


* To run the tool: Select CYW920819EVB-02 application -> Select Project Tab -> Select ModusToolbox -> Select Power Estimator


* To get Power Esimator User Guide: In the tool go to Help -> View Help


* The platform power estimator database file is available at:  `wiced\_btsdk\dev-kit\bsp\TARGET\_CYW920819EVB-02\COMPONENT\_wpl\_lib\20819A1\wpl\_power\_data\CYW920819EVB\_02.xml`

### Additional Information

Cypress also offers a purpose-built Bluetooth mesh kit that comes with four boards and several on-board sensors to implement more complete mesh application without any external hardware. To learn more about Cypress Bluetooth mesh evaluation kit, visit [www.cypress.com/CYBT-213043-MESH](https://www.cypress.com/CYBT-213043-MESH).<br/>

The CYW20819 device and the CYW920819EVB-02 evaluation kit are supported in ModusToolbox IDE 1.1 (or later). To learn more about ModusToolbox IDE and download the latest version, please visit the ModusToolbox webpage.

For more information, see [www.cypress.com/CYW920819EVB-02](https://www.cypress.com/CYW920819EVB-02)

Max UART baud rate is 3M

External 32 kHz LPO is used by default. To operate without external LPO, set USE\_32K\_EXTERNAL\_LPO=0 on the make command line or edit the default value in CYW920819EVB-02.mk.
