EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32f103
LIBS:tja1020
LIBS:stm32f103-ibus-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F103RCT6 U?
U 1 1 584E6360
P 5300 3650
F 0 "U?" H 3400 5500 60  0000 C CNN
F 1 "STM32F103RCT6" H 5350 3650 60  0000 C CNN
F 2 "" H 5300 3650 60  0000 C CNN
F 3 "" H 5300 3650 60  0000 C CNN
	1    5300 3650
	1    0    0    -1  
$EndComp
$Comp
L LM7805 U?
U 1 1 584E85BA
P 3700 1000
F 0 "U?" H 3850 804 60  0000 C CNN
F 1 "LM7805" H 3700 1200 60  0000 C CNN
F 2 "" H 3700 1000 60  0000 C CNN
F 3 "" H 3700 1000 60  0000 C CNN
	1    3700 1000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 584E8790
P 2900 700
F 0 "#PWR?" H 2900 650 20  0001 C CNN
F 1 "+12V" H 2900 800 30  0000 C CNN
F 2 "" H 2900 700 60  0000 C CNN
F 3 "" H 2900 700 60  0000 C CNN
	1    2900 700 
	1    0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 584E8873
P 3100 950
F 0 "D?" H 3100 1050 40  0000 C CNN
F 1 "1N4007" H 3100 850 40  0000 C CNN
F 2 "" H 3100 950 60  0000 C CNN
F 3 "" H 3100 950 60  0000 C CNN
	1    3100 950 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 584E895C
P 3300 1150
F 0 "C?" H 3300 1250 40  0000 L CNN
F 1 "470uF" H 3306 1065 40  0000 L CNN
F 2 "" H 3338 1000 30  0000 C CNN
F 3 "" H 3300 1150 60  0000 C CNN
	1    3300 1150
	1    0    0    -1  
$EndComp
$Comp
L C 1uF
U 1 1 584E899B
P 4100 1150
F 0 "1uF" H 4100 1250 40  0000 L CNN
F 1 "C" H 4106 1065 40  0000 L CNN
F 2 "" H 4138 1000 30  0000 C CNN
F 3 "" H 4100 1150 60  0000 C CNN
	1    4100 1150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 584E89EF
P 4500 950
F 0 "#PWR?" H 4500 1040 20  0001 C CNN
F 1 "+5V" H 4500 1040 30  0000 C CNN
F 2 "" H 4500 950 60  0000 C CNN
F 3 "" H 4500 950 60  0000 C CNN
	1    4500 950 
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR?
U 1 1 584E8A80
P 2850 4450
F 0 "#PWR?" H 2850 4410 30  0001 C CNN
F 1 "+3,3V" H 2850 4560 30  0000 C CNN
F 2 "" H 2850 4450 60  0000 C CNN
F 3 "" H 2850 4450 60  0000 C CNN
	1    2850 4450
	1    0    0    -1  
$EndComp
$Comp
L NCP1117ST33T3G U?
U 1 1 584E930C
P 5900 1000
F 0 "U?" H 5900 1250 40  0000 C CNN
F 1 "NCP1117ST33T3G" H 5900 1200 40  0000 C CNN
F 2 "" H 5900 1000 60  0000 C CNN
F 3 "" H 5900 1000 60  0000 C CNN
	1    5900 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 584E945A
P 5000 950
F 0 "#PWR?" H 5000 1040 20  0001 C CNN
F 1 "+5V" H 5000 1040 30  0000 C CNN
F 2 "" H 5000 950 60  0000 C CNN
F 3 "" H 5000 950 60  0000 C CNN
	1    5000 950 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 584E946E
P 5250 1150
F 0 "C?" H 5250 1250 40  0000 L CNN
F 1 "10uF" H 5256 1065 40  0000 L CNN
F 2 "" H 5288 1000 30  0000 C CNN
F 3 "" H 5250 1150 60  0000 C CNN
	1    5250 1150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 584E9499
P 6550 1150
F 0 "C?" H 6550 1250 40  0000 L CNN
F 1 "10uF" H 6556 1065 40  0000 L CNN
F 2 "" H 6588 1000 30  0000 C CNN
F 3 "" H 6550 1150 60  0000 C CNN
	1    6550 1150
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR?
U 1 1 584E94CA
P 6800 950
F 0 "#PWR?" H 6800 910 30  0001 C CNN
F 1 "+3,3V" H 6800 1060 30  0000 C CNN
F 2 "" H 6800 950 60  0000 C CNN
F 3 "" H 6800 950 60  0000 C CNN
	1    6800 950 
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 5850EAA4
P 2400 6100
F 0 "IC?" H 2190 6290 40  0000 C CNN
F 1 "PC817" H 2550 5910 40  0000 C CNN
F 2 "DIP4" H 2200 5920 30  0000 C CIN
F 3 "" H 2400 6100 60  0000 C CNN
	1    2400 6100
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 5850EB21
P 2400 6800
F 0 "IC?" H 2190 6990 40  0000 C CNN
F 1 "PC817" H 2550 6610 40  0000 C CNN
F 2 "DIP4" H 2200 6620 30  0000 C CIN
F 3 "" H 2400 6800 60  0000 C CNN
	1    2400 6800
	-1   0    0    -1  
$EndComp
Text Label 950  6400 0    60   ~ 0
iBus
$Comp
L R R?
U 1 1 5850EEAB
P 1650 6000
F 0 "R?" V 1730 6000 40  0000 C CNN
F 1 "2K" V 1657 6001 40  0000 C CNN
F 2 "" V 1580 6000 30  0000 C CNN
F 3 "" H 1650 6000 30  0000 C CNN
	1    1650 6000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5850F037
P 2050 6450
F 0 "#PWR?" H 2050 6450 30  0001 C CNN
F 1 "GND" H 2050 6380 30  0001 C CNN
F 2 "" H 2050 6450 60  0000 C CNN
F 3 "" H 2050 6450 60  0000 C CNN
	1    2050 6450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5850F0CA
P 2850 5800
F 0 "#PWR?" H 2850 5760 30  0001 C CNN
F 1 "+3.3V" H 2850 5910 30  0000 C CNN
F 2 "" H 2850 5800 60  0000 C CNN
F 3 "" H 2850 5800 60  0000 C CNN
	1    2850 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5850F48F
P 1900 7300
F 0 "#PWR?" H 1900 7300 30  0001 C CNN
F 1 "GND" H 1900 7230 30  0001 C CNN
F 2 "" H 1900 7300 60  0000 C CNN
F 3 "" H 1900 7300 60  0000 C CNN
	1    1900 7300
	1    0    0    -1  
$EndComp
$Comp
L S8050 Q?
U 1 1 5850F745
P 3050 6900
F 0 "Q?" H 3050 6751 40  0000 R CNN
F 1 "S8050" H 3050 7050 40  0000 R CNN
F 2 "TO92" H 2950 7002 29  0000 C CNN
F 3 "" H 3050 6900 60  0000 C CNN
	1    3050 6900
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5850F9E0
P 3550 6700
F 0 "R?" V 3630 6700 40  0000 C CNN
F 1 "470" V 3557 6701 40  0000 C CNN
F 2 "" V 3480 6700 30  0000 C CNN
F 3 "" H 3550 6700 30  0000 C CNN
	1    3550 6700
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5850FB0B
P 4050 6700
F 0 "#PWR?" H 4050 6660 30  0001 C CNN
F 1 "+3.3V" H 4050 6810 30  0000 C CNN
F 2 "" H 4050 6700 60  0000 C CNN
F 3 "" H 4050 6700 60  0000 C CNN
	1    4050 6700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5850FB63
P 3550 7200
F 0 "R?" V 3630 7200 40  0000 C CNN
F 1 "10k" V 3557 7201 40  0000 C CNN
F 2 "" V 3480 7200 30  0000 C CNN
F 3 "" H 3550 7200 30  0000 C CNN
	1    3550 7200
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 5850FC73
P 4050 6950
F 0 "R?" V 4130 6950 40  0000 C CNN
F 1 "1K" V 4057 6951 40  0000 C CNN
F 2 "" V 3980 6950 30  0000 C CNN
F 3 "" H 4050 6950 30  0000 C CNN
	1    4050 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5850FD89
P 2950 7350
F 0 "#PWR?" H 2950 7350 30  0001 C CNN
F 1 "GND" H 2950 7280 30  0001 C CNN
F 2 "" H 2950 7350 60  0000 C CNN
F 3 "" H 2950 7350 60  0000 C CNN
	1    2950 7350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5850FE2E
P 3500 6450
F 0 "#PWR?" H 3500 6450 30  0001 C CNN
F 1 "GND" H 3500 6380 30  0001 C CNN
F 2 "" H 3500 6450 60  0000 C CNN
F 3 "" H 3500 6450 60  0000 C CNN
	1    3500 6450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5850FE42
P 3100 6200
F 0 "R?" V 3180 6200 40  0000 C CNN
F 1 "470" V 3107 6201 40  0000 C CNN
F 2 "" V 3030 6200 30  0000 C CNN
F 3 "" H 3100 6200 30  0000 C CNN
	1    3100 6200
	0    1    1    0   
$EndComp
Text GLabel 3900 6200 2    60   Input ~ 0
LIN_RX
Text GLabel 4450 7200 2    60   Input ~ 0
LIN_TX
Text GLabel 7650 3150 2    60   Input ~ 0
LIN_RX
Text GLabel 7650 3050 2    60   Input ~ 0
LIN_TX
$Comp
L +3.3V #PWR?
U 1 1 585237F6
P 7250 950
F 0 "#PWR?" H 7250 910 30  0001 C CNN
F 1 "+3.3V" H 7250 1060 30  0000 C CNN
F 2 "" H 7250 950 60  0000 C CNN
F 3 "" H 7250 950 60  0000 C CNN
	1    7250 950 
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5852380A
P 7500 1150
F 0 "C?" H 7500 1250 40  0000 L CNN
F 1 "0.1uF" H 7506 1065 40  0000 L CNN
F 2 "" H 7538 1000 30  0000 C CNN
F 3 "" H 7500 1150 60  0000 C CNN
	1    7500 1150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5852383B
P 7750 1150
F 0 "C?" H 7750 1250 40  0000 L CNN
F 1 "0.1uF" H 7756 1065 40  0000 L CNN
F 2 "" H 7788 1000 30  0000 C CNN
F 3 "" H 7750 1150 60  0000 C CNN
	1    7750 1150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5852388C
P 8000 1150
F 0 "C?" H 8000 1250 40  0000 L CNN
F 1 "0.1uF" H 8006 1065 40  0000 L CNN
F 2 "" H 8038 1000 30  0000 C CNN
F 3 "" H 8000 1150 60  0000 C CNN
	1    8000 1150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585238B5
P 8250 1150
F 0 "C?" H 8250 1250 40  0000 L CNN
F 1 "0.1uF" H 8256 1065 40  0000 L CNN
F 2 "" H 8288 1000 30  0000 C CNN
F 3 "" H 8250 1150 60  0000 C CNN
	1    8250 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58523CD8
P 7250 1450
F 0 "#PWR?" H 7250 1450 30  0001 C CNN
F 1 "GND" H 7250 1380 30  0001 C CNN
F 2 "" H 7250 1450 60  0000 C CNN
F 3 "" H 7250 1450 60  0000 C CNN
	1    7250 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5852456A
P 3050 5450
F 0 "#PWR?" H 3050 5450 30  0001 C CNN
F 1 "GND" H 3050 5380 30  0001 C CNN
F 2 "" H 3050 5450 60  0000 C CNN
F 3 "" H 3050 5450 60  0000 C CNN
	1    3050 5450
	1    0    0    -1  
$EndComp
Text GLabel 2450 2250 0    60   Input ~ 0
USART2_TX
Text GLabel 2450 2350 0    60   Input ~ 0
USART2_RX
Text GLabel 9300 3750 2    60   Input ~ 0
COM
Text GLabel 9300 3550 2    60   Input ~ 0
BTN_NEXT
Text GLabel 9300 4050 2    60   Input ~ 0
BTN_PREV
Text GLabel 9300 4250 2    60   Input ~ 0
COM
Text GLabel 9300 4550 2    60   Input ~ 0
BTN_PLAY
Text GLabel 9300 4750 2    60   Input ~ 0
COM
$Comp
L +3.3V #PWR?
U 1 1 58528B82
P 7900 3550
F 0 "#PWR?" H 7900 3510 30  0001 C CNN
F 1 "+3.3V" H 7900 3660 30  0000 C CNN
F 2 "" H 7900 3550 60  0000 C CNN
F 3 "" H 7900 3550 60  0000 C CNN
	1    7900 3550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58528B96
P 8150 3550
F 0 "R?" V 8230 3550 40  0000 C CNN
F 1 "1k" V 8157 3551 40  0000 C CNN
F 2 "" V 8080 3550 30  0000 C CNN
F 3 "" H 8150 3550 30  0000 C CNN
	1    8150 3550
	0    1    1    0   
$EndComp
$Comp
L PC817 IC?
U 1 1 58528F6C
P 8750 3650
F 0 "IC?" H 8540 3840 40  0000 C CNN
F 1 "PC817" H 8900 3460 40  0000 C CNN
F 2 "DIP4" H 8550 3470 30  0000 C CIN
F 3 "" H 8750 3650 60  0000 C CNN
	1    8750 3650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5852908F
P 7900 4050
F 0 "#PWR?" H 7900 4010 30  0001 C CNN
F 1 "+3.3V" H 7900 4160 30  0000 C CNN
F 2 "" H 7900 4050 60  0000 C CNN
F 3 "" H 7900 4050 60  0000 C CNN
	1    7900 4050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58529095
P 8150 4050
F 0 "R?" V 8230 4050 40  0000 C CNN
F 1 "1k" V 8157 4051 40  0000 C CNN
F 2 "" V 8080 4050 30  0000 C CNN
F 3 "" H 8150 4050 30  0000 C CNN
	1    8150 4050
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5852914B
P 7900 4550
F 0 "#PWR?" H 7900 4510 30  0001 C CNN
F 1 "+3.3V" H 7900 4660 30  0000 C CNN
F 2 "" H 7900 4550 60  0000 C CNN
F 3 "" H 7900 4550 60  0000 C CNN
	1    7900 4550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58529151
P 8150 4550
F 0 "R?" V 8230 4550 40  0000 C CNN
F 1 "1k" V 8157 4551 40  0000 C CNN
F 2 "" V 8080 4550 30  0000 C CNN
F 3 "" H 8150 4550 30  0000 C CNN
	1    8150 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 700  2900 950 
Wire Wire Line
	4500 950  4100 950 
Wire Wire Line
	6300 950  6800 950 
Connection ~ 6550 950 
Wire Wire Line
	5000 950  5500 950 
Connection ~ 5250 950 
Wire Bus Line
	950  5400 950  7450
Wire Wire Line
	950  6000 1400 6000
Wire Wire Line
	1900 6000 2050 6000
Wire Wire Line
	2050 6200 2050 6450
Wire Wire Line
	2850 5800 2850 6000
Wire Wire Line
	2850 6000 2750 6000
Wire Wire Line
	2050 6900 1900 6900
Wire Wire Line
	1900 6900 1900 7300
Wire Wire Line
	1400 6000 1400 6700
Wire Wire Line
	1400 6700 2050 6700
Wire Wire Line
	2750 6700 3300 6700
Connection ~ 2950 6700
Wire Wire Line
	4050 6700 3800 6700
Wire Wire Line
	3800 7200 4450 7200
Wire Wire Line
	2950 7100 2950 7350
Wire Wire Line
	3250 6900 3250 7200
Wire Wire Line
	3250 7200 3300 7200
Wire Wire Line
	2750 6200 2850 6200
Wire Wire Line
	3500 6200 3500 6450
Connection ~ 3500 6200
Wire Wire Line
	2750 6900 2750 7100
Wire Wire Line
	2750 7100 2950 7100
Connection ~ 4050 7200
Wire Wire Line
	3350 6200 3900 6200
Wire Wire Line
	7250 950  8250 950 
Connection ~ 7500 950 
Connection ~ 7750 950 
Connection ~ 8000 950 
Wire Wire Line
	7250 1350 8250 1350
Connection ~ 8000 1350
Connection ~ 7750 1350
Wire Wire Line
	7250 1450 7250 1350
Connection ~ 7500 1350
Wire Wire Line
	3050 5350 3200 5350
Wire Wire Line
	3050 4950 3050 5450
Wire Wire Line
	3200 5250 3050 5250
Connection ~ 3050 5350
Wire Wire Line
	3200 5150 3050 5150
Connection ~ 3050 5250
Wire Wire Line
	3200 5050 3050 5050
Connection ~ 3050 5150
Wire Wire Line
	3200 4950 3050 4950
Connection ~ 3050 5050
Wire Wire Line
	2850 4450 3200 4450
Wire Wire Line
	3200 4550 3050 4550
Wire Wire Line
	3050 4450 3050 4850
Connection ~ 3050 4450
Wire Wire Line
	3050 4650 3200 4650
Connection ~ 3050 4550
Wire Wire Line
	3050 4750 3200 4750
Connection ~ 3050 4650
Wire Wire Line
	3050 4850 3200 4850
Connection ~ 3050 4750
Wire Wire Line
	2450 2250 3200 2250
Wire Wire Line
	2450 2350 3200 2350
Wire Wire Line
	9100 3550 9300 3550
Wire Wire Line
	9100 3750 9300 3750
Wire Wire Line
	7400 3050 7650 3050
Wire Wire Line
	7400 3150 7650 3150
Wire Wire Line
	9100 4050 9300 4050
Wire Wire Line
	9100 4250 9300 4250
Wire Wire Line
	9100 4550 9300 4550
Wire Wire Line
	9100 4750 9300 4750
Wire Wire Line
	7400 3750 8400 3750
$Comp
L PC817 IC?
U 1 1 585292C5
P 8750 4150
F 0 "IC?" H 8540 4340 40  0000 C CNN
F 1 "PC817" H 8900 3960 40  0000 C CNN
F 2 "DIP4" H 8550 3970 30  0000 C CIN
F 3 "" H 8750 4150 60  0000 C CNN
	1    8750 4150
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 58529331
P 8750 4650
F 0 "IC?" H 8540 4840 40  0000 C CNN
F 1 "PC817" H 8900 4460 40  0000 C CNN
F 2 "DIP4" H 8550 4470 30  0000 C CIN
F 3 "" H 8750 4650 60  0000 C CNN
	1    8750 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3850 7750 3850
Wire Wire Line
	7750 3850 7750 4250
Wire Wire Line
	7750 4250 8400 4250
Wire Wire Line
	7400 3950 7650 3950
Wire Wire Line
	7650 3950 7650 4750
Wire Wire Line
	7650 4750 8400 4750
$EndSCHEMATC
