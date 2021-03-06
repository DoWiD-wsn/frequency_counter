EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "AVR-based Frequency Counter"
Date "2022-02-26"
Rev "1.0"
Comp "UAS Technikum Wien"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATmega:ATmega8-16AU U1
U 1 1 621A33BF
P 8000 3400
F 0 "U1" H 7550 4750 50  0000 C CNN
F 1 "ATmega8-16AU" V 8000 3400 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 8000 3400 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2486-8-bit-avr-microcontroller-atmega8_l_datasheet.pdf" H 8000 3400 50  0001 C CNN
	1    8000 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 621A46F6
P 7300 2150
F 0 "R1" H 7150 2200 50  0000 L CNN
F 1 "4k7" H 7100 2100 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7230 2150 50  0001 C CNN
F 3 "~" H 7300 2150 50  0001 C CNN
	1    7300 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector:AVR-ISP-6 J4
U 1 1 621A4DC7
P 5700 2600
F 0 "J4" H 5900 3100 50  0000 R CNN
F 1 "AVR-ISP-6" H 6100 2200 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" V 5450 2650 50  0001 C CNN
F 3 " ~" H 4425 2050 50  0001 C CNN
	1    5700 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 621A5C18
P 7200 2600
F 0 "Y1" V 7200 2450 50  0000 R CNN
F 1 "4.096M" H 7350 2750 50  0000 R CNN
F 2 "Crystal:Crystal_SMD_HC49-SD" H 7200 2600 50  0001 C CNN
F 3 "~" H 7200 2600 50  0001 C CNN
	1    7200 2600
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C8
U 1 1 621A65F7
P 6900 2450
F 0 "C8" V 6850 2300 50  0000 C CNN
F 1 "18p" V 7000 2300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6938 2300 50  0001 C CNN
F 3 "~" H 6900 2450 50  0001 C CNN
	1    6900 2450
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 621A6CA8
P 6900 2750
F 0 "C9" V 6850 2600 50  0000 C CNN
F 1 "18p" V 7000 2600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6938 2600 50  0001 C CNN
F 3 "~" H 6900 2750 50  0001 C CNN
	1    6900 2750
	0    1    1    0   
$EndComp
Text Notes 2650 3050 2    50   ~ 0
GND
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 621A9C9D
P 2800 2400
F 0 "J1" H 2750 2600 50  0000 L CNN
F 1 "PWR/I2C" V 2900 2200 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2800 2400 50  0001 C CNN
F 3 "~" H 2800 2400 50  0001 C CNN
	1    2800 2400
	-1   0    0    -1  
$EndComp
Text Notes 2650 2350 2    50   ~ 0
GND
$Comp
L power:+5V #PWR03
U 1 1 621AAC60
P 3200 2200
F 0 "#PWR03" H 3200 2050 50  0001 C CNN
F 1 "+5V" H 3215 2373 50  0000 C CNN
F 2 "" H 3200 2200 50  0001 C CNN
F 3 "" H 3200 2200 50  0001 C CNN
	1    3200 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 621AB1A2
P 3100 3600
F 0 "#PWR01" H 3100 3350 50  0001 C CNN
F 1 "GND" H 3105 3427 50  0000 C CNN
F 2 "" H 3100 3600 50  0001 C CNN
F 3 "" H 3100 3600 50  0001 C CNN
	1    3100 3600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 621ABC2D
P 3300 2200
F 0 "#FLG01" H 3300 2275 50  0001 C CNN
F 1 "PWR_FLAG" H 3550 2300 50  0000 C CNN
F 2 "" H 3300 2200 50  0001 C CNN
F 3 "~" H 3300 2200 50  0001 C CNN
	1    3300 2200
	1    0    0    -1  
$EndComp
Text Notes 2500 5700 0    50   ~ 0
CH0
$Comp
L Connector_Generic:Conn_01x06 J2
U 1 1 621AF248
P 2800 3200
F 0 "J2" H 2750 3500 50  0000 L CNN
F 1 "FTDI" V 2900 3100 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 3200 50  0001 C CNN
F 3 "~" H 2800 3200 50  0001 C CNN
	1    2800 3200
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J3
U 1 1 621B115F
P 2800 5550
F 0 "J3" H 2750 5850 50  0000 L CNN
F 1 "Input" V 2900 5450 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 5550 50  0001 C CNN
F 3 "~" H 2800 5550 50  0001 C CNN
	1    2800 5550
	-1   0    0    1   
$EndComp
Text Notes 2650 3150 2    50   ~ 0
CTS
Text Notes 2650 3250 2    50   ~ 0
VCC
Text Notes 2650 3350 2    50   ~ 0
TXD
Text Notes 2650 3450 2    50   ~ 0
RXD
Text Notes 2650 3550 2    50   ~ 0
RTS
Text Notes 2650 2450 2    50   ~ 0
VCC
Text Notes 2650 2550 2    50   ~ 0
SDA
Text Notes 2650 2650 2    50   ~ 0
SCL
Text Notes 2500 5300 0    50   ~ 0
GND
Text Notes 2500 5600 0    50   ~ 0
CH1
Text Notes 2500 5500 0    50   ~ 0
CH2
Text Notes 2500 5800 0    50   ~ 0
GND
Text Notes 2500 5400 0    50   ~ 0
CH3
$Comp
L mysymbols:SN74HC00PW IC4
U 1 1 621C0B7A
P 4750 5250
F 0 "IC4" H 5000 5400 50  0000 C CNN
F 1 "SN74HC00PW" H 5250 4500 50  0000 C CNN
F 2 "myfootprint:SOP65P640X120-14N" H 5600 5350 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/SN74HC00" H 5600 5250 50  0001 L CNN
	1    4750 5250
	1    0    0    -1  
$EndComp
$Comp
L mysymbols:SN74HC4040PW IC1
U 1 1 621C3376
P 3400 3900
F 0 "IC1" H 3650 4050 50  0000 C CNN
F 1 "SN74HC4040PW" H 3900 3050 50  0000 C CNN
F 2 "myfootprint:SOP65P640X120-16N" H 4250 4000 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc4040.pdf" H 4250 3900 50  0001 L CNN
	1    3400 3900
	1    0    0    -1  
$EndComp
$Comp
L mysymbols:SN74HC4040PW IC3
U 1 1 621C522B
P 4750 3900
F 0 "IC3" H 5000 4050 50  0000 C CNN
F 1 "SN74HC4040PW" H 5250 3050 50  0000 C CNN
F 2 "myfootprint:SOP65P640X120-16N" H 5600 4000 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc4040.pdf" H 5600 3900 50  0001 L CNN
	1    4750 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 621F066E
P 3100 5850
F 0 "#PWR02" H 3100 5600 50  0001 C CNN
F 1 "GND" H 3100 5700 50  0000 C CNN
F 2 "" H 3100 5850 50  0001 C CNN
F 3 "" H 3100 5850 50  0001 C CNN
	1    3100 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5250 3100 5250
Wire Wire Line
	3100 5250 3100 5750
Wire Wire Line
	3000 5750 3100 5750
Connection ~ 3100 5750
Wire Wire Line
	3100 5750 3100 5850
NoConn ~ 4400 5850
NoConn ~ 4400 5750
NoConn ~ 4400 5650
NoConn ~ 4400 5550
NoConn ~ 4400 5450
NoConn ~ 4400 5250
$Comp
L power:+5V #PWR08
U 1 1 621F432C
P 4400 5150
F 0 "#PWR08" H 4400 5000 50  0001 C CNN
F 1 "+5V" H 4400 5300 50  0000 C CNN
F 2 "" H 4400 5150 50  0001 C CNN
F 3 "" H 4400 5150 50  0001 C CNN
	1    4400 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 5150 3100 5250
Connection ~ 3100 5250
Text Label 3400 5750 2    50   ~ 0
Y
Text Label 4750 5250 2    50   ~ 0
Y
$Comp
L power:+5V #PWR014
U 1 1 62205E8D
P 5750 5250
F 0 "#PWR014" H 5750 5100 50  0001 C CNN
F 1 "+5V" H 5750 5400 50  0000 C CNN
F 2 "" H 5750 5250 50  0001 C CNN
F 3 "" H 5750 5250 50  0001 C CNN
	1    5750 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 62206869
P 4750 5850
F 0 "#PWR010" H 4750 5600 50  0001 C CNN
F 1 "GND" H 4750 5700 50  0000 C CNN
F 2 "" H 4750 5850 50  0001 C CNN
F 3 "" H 4750 5850 50  0001 C CNN
	1    4750 5850
	1    0    0    -1  
$EndComp
Text Label 4750 5350 2    50   ~ 0
Y
Wire Wire Line
	4750 5450 4750 5550
Text Label 4750 5750 2    50   ~ 0
CLK
Text Label 4750 5650 2    50   ~ 0
GATE
NoConn ~ 5750 5850
NoConn ~ 5750 5750
NoConn ~ 5750 5650
NoConn ~ 5750 5550
NoConn ~ 5750 5450
NoConn ~ 5750 5350
Text Label 5750 4200 0    50   ~ 0
GATE
Text Label 7200 4500 0    50   ~ 0
CLK
$Comp
L power:GND #PWR06
U 1 1 62215102
P 3400 4600
F 0 "#PWR06" H 3400 4350 50  0001 C CNN
F 1 "GND" H 3400 4450 50  0000 C CNN
F 2 "" H 3400 4600 50  0001 C CNN
F 3 "" H 3400 4600 50  0001 C CNN
	1    3400 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 622159AA
P 4750 4600
F 0 "#PWR09" H 4750 4350 50  0001 C CNN
F 1 "GND" H 4750 4450 50  0000 C CNN
F 2 "" H 4750 4600 50  0001 C CNN
F 3 "" H 4750 4600 50  0001 C CNN
	1    4750 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 62215FDB
P 5750 3900
F 0 "#PWR013" H 5750 3750 50  0001 C CNN
F 1 "+5V" H 5750 4050 50  0000 C CNN
F 2 "" H 5750 3900 50  0001 C CNN
F 3 "" H 5750 3900 50  0001 C CNN
	1    5750 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 622168CB
P 4400 3900
F 0 "#PWR07" H 4400 3750 50  0001 C CNN
F 1 "+5V" H 4400 4050 50  0000 C CNN
F 2 "" H 4400 3900 50  0001 C CNN
F 3 "" H 4400 3900 50  0001 C CNN
	1    4400 3900
	1    0    0    -1  
$EndComp
Text Label 4400 4400 0    50   ~ 0
PC1
Text Label 5750 4400 0    50   ~ 0
PC1
NoConn ~ 3400 4000
NoConn ~ 3400 4100
NoConn ~ 3400 4200
NoConn ~ 3400 4300
NoConn ~ 3400 4400
NoConn ~ 3400 4500
NoConn ~ 4400 4600
NoConn ~ 4400 4000
NoConn ~ 4400 4100
NoConn ~ 4400 4200
NoConn ~ 4400 4300
Text Label 3400 3900 2    50   ~ 0
1K
Text Label 5750 4500 0    50   ~ 0
1K
NoConn ~ 5750 4600
NoConn ~ 5750 4300
NoConn ~ 5750 4100
NoConn ~ 5750 4000
NoConn ~ 4750 4500
NoConn ~ 4750 4400
NoConn ~ 4750 4300
NoConn ~ 4750 4200
NoConn ~ 4750 4100
NoConn ~ 4750 4000
NoConn ~ 4750 3900
Text Label 4400 4500 0    50   ~ 0
4M
Text Label 7200 2450 2    50   ~ 0
4M
$Comp
L power:+5V #PWR017
U 1 1 62237DF0
P 7200 3900
F 0 "#PWR017" H 7200 3750 50  0001 C CNN
F 1 "+5V" H 7200 4050 50  0000 C CNN
F 2 "" H 7200 3900 50  0001 C CNN
F 3 "" H 7200 3900 50  0001 C CNN
	1    7200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 62238473
P 6200 4600
F 0 "#PWR015" H 6200 4350 50  0001 C CNN
F 1 "GND" H 6200 4450 50  0000 C CNN
F 2 "" H 6200 4600 50  0001 C CNN
F 3 "" H 6200 4600 50  0001 C CNN
	1    6200 4600
	1    0    0    -1  
$EndComp
Text Label 7200 4600 0    50   ~ 0
PB0
$Comp
L mysymbols:SN74HC4040PW IC5
U 1 1 621C5B9C
P 6200 3900
F 0 "IC5" H 6450 4050 50  0000 C CNN
F 1 "SN74HC4040PW" H 6700 3050 50  0000 C CNN
F 2 "myfootprint:SOP65P640X120-16N" H 7050 4000 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc4040.pdf" H 7050 3900 50  0001 L CNN
	1    6200 3900
	1    0    0    -1  
$EndComp
Text Label 6200 4500 2    50   ~ 0
PB1
Text Label 6200 4400 2    50   ~ 0
PB2
Text Label 6200 4300 2    50   ~ 0
PD7
Text Label 6200 4100 2    50   ~ 0
PD6
Text Label 6200 4000 2    50   ~ 0
PD3
Text Label 7200 4200 0    50   ~ 0
T1
NoConn ~ 7200 4300
NoConn ~ 7200 4100
NoConn ~ 7200 4000
Text Label 6200 4200 2    50   ~ 0
PD4
$Comp
L power:GND #PWR020
U 1 1 6228F69A
P 8000 4900
F 0 "#PWR020" H 8000 4650 50  0001 C CNN
F 1 "GND" H 8000 4750 50  0000 C CNN
F 2 "" H 8000 4900 50  0001 C CNN
F 3 "" H 8000 4900 50  0001 C CNN
	1    8000 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4800 8000 4800
Wire Wire Line
	8000 4800 8000 4900
Connection ~ 8000 4800
$Comp
L power:GND #PWR016
U 1 1 622A876C
P 6600 2850
F 0 "#PWR016" H 6600 2600 50  0001 C CNN
F 1 "GND" H 6600 2700 50  0000 C CNN
F 2 "" H 6600 2850 50  0001 C CNN
F 3 "" H 6600 2850 50  0001 C CNN
	1    6600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2450 7200 2450
Wire Wire Line
	7200 2450 7400 2450
Wire Wire Line
	7400 2450 7400 2500
Connection ~ 7200 2450
Wire Wire Line
	7050 2750 7200 2750
Wire Wire Line
	7200 2750 7400 2750
Wire Wire Line
	7400 2750 7400 2700
Connection ~ 7200 2750
Wire Wire Line
	6750 2450 6600 2450
Wire Wire Line
	6600 2450 6600 2750
Connection ~ 6600 2750
Wire Wire Line
	6600 2750 6600 2850
Wire Wire Line
	6600 2750 6750 2750
Wire Wire Line
	3000 5350 3400 5350
Wire Wire Line
	3000 5450 3400 5450
Wire Wire Line
	3000 5550 3400 5550
Wire Wire Line
	3000 5650 3400 5650
Wire Wire Line
	3400 5850 3100 5850
Connection ~ 3100 5850
Text Label 4400 5350 0    50   ~ 0
PC3
Wire Wire Line
	3100 5150 3400 5150
$Comp
L mysymbols:SN74HC153PW IC2
U 1 1 621C251D
P 3400 5150
F 0 "IC2" H 3650 5300 50  0000 C CNN
F 1 "SN74HC153PW" H 3900 4300 50  0000 C CNN
F 2 "myfootprint:SOP65P640X120-16N" H 4250 5250 50  0001 L CNN
F 3 "http://www.ti.com/lit/gpn/SN74HC153" H 4250 5150 50  0001 L CNN
	1    3400 5150
	1    0    0    -1  
$EndComp
Text Label 3400 5250 2    50   ~ 0
PC2
$Comp
L power:+5V #PWR019
U 1 1 622D0226
P 8000 1900
F 0 "#PWR019" H 8000 1750 50  0001 C CNN
F 1 "+5V" H 8015 2073 50  0000 C CNN
F 2 "" H 8000 1900 50  0001 C CNN
F 3 "" H 8000 1900 50  0001 C CNN
	1    8000 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2000 8000 2000
Wire Wire Line
	8000 2000 8000 1900
Connection ~ 8000 2000
Wire Wire Line
	8000 2000 7300 2000
Wire Wire Line
	7300 2300 7400 2300
$Comp
L power:+5V #PWR018
U 1 1 622D56E8
P 7400 2900
F 0 "#PWR018" H 7400 2750 50  0001 C CNN
F 1 "+5V" V 7415 3028 50  0000 L CNN
F 2 "" H 7400 2900 50  0001 C CNN
F 3 "" H 7400 2900 50  0001 C CNN
	1    7400 2900
	0    -1   -1   0   
$EndComp
NoConn ~ 7400 3100
NoConn ~ 7400 3200
Text Label 8600 2300 0    50   ~ 0
PB0
Text Label 8600 2400 0    50   ~ 0
PB1
Text Label 8600 2500 0    50   ~ 0
PB2
Text Label 8600 3000 0    50   ~ 0
PC0
Text Label 8600 3100 0    50   ~ 0
PC1
Text Label 8600 3300 0    50   ~ 0
PC3
Text Label 8600 3200 0    50   ~ 0
PC2
Text Label 8600 3400 0    50   ~ 0
SDA
Text Label 8600 3500 0    50   ~ 0
SCL
Text Label 8600 3700 0    50   ~ 0
RXD
Text Label 8600 3800 0    50   ~ 0
TXD
Text Label 8600 3900 0    50   ~ 0
INT0
Text Label 8600 4000 0    50   ~ 0
PD3
Text Label 8600 4100 0    50   ~ 0
PD4
Text Label 8600 4200 0    50   ~ 0
T1
Text Label 8600 4300 0    50   ~ 0
PD6
Text Label 8600 4400 0    50   ~ 0
PD7
Text Label 7200 4400 0    50   ~ 0
PC0
Text Label 5150 5000 2    50   ~ 0
GATE
Text Label 5350 5000 0    50   ~ 0
INT0
Wire Wire Line
	5150 5000 5350 5000
Text Label 8600 2600 0    50   ~ 0
MOSI
Text Label 8600 2700 0    50   ~ 0
MISO
Text Label 8600 2800 0    50   ~ 0
SCK
$Comp
L power:GND #PWR012
U 1 1 62306DAB
P 5600 3000
F 0 "#PWR012" H 5600 2750 50  0001 C CNN
F 1 "GND" H 5605 2827 50  0000 C CNN
F 2 "" H 5600 3000 50  0001 C CNN
F 3 "" H 5600 3000 50  0001 C CNN
	1    5600 3000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 62307216
P 5600 2100
F 0 "#PWR011" H 5600 1950 50  0001 C CNN
F 1 "+5V" H 5615 2273 50  0000 C CNN
F 2 "" H 5600 2100 50  0001 C CNN
F 3 "" H 5600 2100 50  0001 C CNN
	1    5600 2100
	1    0    0    -1  
$EndComp
Text Label 6100 2500 0    50   ~ 0
MOSI
Text Label 6100 2400 0    50   ~ 0
MISO
Text Label 6100 2600 0    50   ~ 0
SCK
Text Label 6100 2700 0    50   ~ 0
RST
Text Label 7100 2300 2    50   ~ 0
RST
Wire Wire Line
	7100 2300 7300 2300
Connection ~ 7300 2300
Text Label 3000 2500 0    50   ~ 0
SDA
Text Label 3000 2600 0    50   ~ 0
SCL
Wire Wire Line
	3000 2300 3100 2300
Wire Wire Line
	3100 2300 3100 3000
Wire Wire Line
	3000 3000 3100 3000
Connection ~ 3100 3000
Wire Wire Line
	3100 3000 3100 3600
NoConn ~ 3000 3100
NoConn ~ 3000 3500
Text Label 3000 3300 0    50   ~ 0
RXD
Text Label 3000 3400 0    50   ~ 0
TXD
Wire Wire Line
	3200 2200 3200 2400
Wire Wire Line
	3200 3200 3000 3200
Wire Wire Line
	3000 2400 3200 2400
Connection ~ 3200 2400
Wire Wire Line
	3200 2400 3200 3200
Wire Wire Line
	3300 2200 3200 2200
Connection ~ 3200 2200
$Comp
L power:PWR_FLAG #FLG02
U 1 1 6235D3B9
P 3300 3600
F 0 "#FLG02" H 3300 3675 50  0001 C CNN
F 1 "PWR_FLAG" H 3550 3700 50  0000 C CNN
F 2 "" H 3300 3600 50  0001 C CNN
F 3 "~" H 3300 3600 50  0001 C CNN
	1    3300 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3600 3100 3600
Connection ~ 3100 3600
NoConn ~ 6200 3900
$Comp
L Device:C C1
U 1 1 623644BE
P 3400 2800
F 0 "C1" H 3350 2700 50  0000 C CNN
F 1 "100n" H 3300 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3438 2650 50  0001 C CNN
F 3 "~" H 3400 2800 50  0001 C CNN
	1    3400 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 623742C3
P 3650 2800
F 0 "C2" H 3600 2700 50  0000 C CNN
F 1 "100n" H 3550 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3688 2650 50  0001 C CNN
F 3 "~" H 3650 2800 50  0001 C CNN
	1    3650 2800
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 62375704
P 3400 2650
F 0 "#PWR04" H 3400 2500 50  0001 C CNN
F 1 "+5V" H 3415 2823 50  0000 C CNN
F 2 "" H 3400 2650 50  0001 C CNN
F 3 "" H 3400 2650 50  0001 C CNN
	1    3400 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6237DAE5
P 3400 2950
F 0 "#PWR05" H 3400 2700 50  0001 C CNN
F 1 "GND" H 3405 2777 50  0000 C CNN
F 2 "" H 3400 2950 50  0001 C CNN
F 3 "" H 3400 2950 50  0001 C CNN
	1    3400 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 6237E11C
P 3900 2800
F 0 "C3" H 3850 2700 50  0000 C CNN
F 1 "100n" H 3800 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3938 2650 50  0001 C CNN
F 3 "~" H 3900 2800 50  0001 C CNN
	1    3900 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C4
U 1 1 6237E94C
P 4150 2800
F 0 "C4" H 4100 2700 50  0000 C CNN
F 1 "100n" H 4050 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4188 2650 50  0001 C CNN
F 3 "~" H 4150 2800 50  0001 C CNN
	1    4150 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C5
U 1 1 6237EFC3
P 4400 2800
F 0 "C5" H 4350 2700 50  0000 C CNN
F 1 "100n" H 4300 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4438 2650 50  0001 C CNN
F 3 "~" H 4400 2800 50  0001 C CNN
	1    4400 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	3400 2650 3650 2650
Connection ~ 3400 2650
Connection ~ 3650 2650
Wire Wire Line
	3650 2650 3900 2650
Connection ~ 3900 2650
Wire Wire Line
	3900 2650 4150 2650
Connection ~ 4150 2650
Wire Wire Line
	4150 2650 4400 2650
Wire Wire Line
	4400 2950 4150 2950
Connection ~ 3400 2950
Connection ~ 3650 2950
Wire Wire Line
	3650 2950 3400 2950
Connection ~ 3900 2950
Wire Wire Line
	3900 2950 3650 2950
Connection ~ 4150 2950
Wire Wire Line
	4150 2950 3900 2950
$Comp
L Device:C C6
U 1 1 62395DEE
P 4650 2800
F 0 "C6" H 4600 2700 50  0000 C CNN
F 1 "100n" H 4550 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4688 2650 50  0001 C CNN
F 3 "~" H 4650 2800 50  0001 C CNN
	1    4650 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 62396AB3
P 4900 2800
F 0 "C7" H 4850 2700 50  0000 C CNN
F 1 "100n" H 4800 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4938 2650 50  0001 C CNN
F 3 "~" H 4900 2800 50  0001 C CNN
	1    4900 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 2650 4650 2650
Connection ~ 4400 2650
Connection ~ 4650 2650
Wire Wire Line
	4650 2650 4900 2650
Wire Wire Line
	4400 2950 4650 2950
Connection ~ 4400 2950
Connection ~ 4650 2950
Wire Wire Line
	4650 2950 4900 2950
$EndSCHEMATC
