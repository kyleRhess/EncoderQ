EESchema Schematic File Version 4
EELAYER 30 0
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
L MCU_ST_STM32F4:STM32F410RBTx U3
U 1 1 5BBEB43A
P 9100 2700
F 0 "U3" H 9550 950 50  0000 C CNN
F 1 "STM32F410RBTx" H 9750 4450 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 8500 1000 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00214043.pdf" H 9100 2700 50  0001 C CNN
	1    9100 2700
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  550  2800 550 
Wire Notes Line
	2800 2100 550  2100
Wire Notes Line
	550  2100 550  550 
$Comp
L Regulator_Linear:LP2985-3.3 U1
U 1 1 5BBEBD46
P 1700 1000
F 0 "U1" H 1700 1342 50  0000 C CNN
F 1 "LP2985-3.3" H 1700 1251 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 1700 1325 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lp2985.pdf" H 1700 1000 50  0001 C CNN
	1    1700 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 900  1250 900 
Wire Wire Line
	1250 900  1250 1000
Wire Wire Line
	1250 1000 1300 1000
$Comp
L power:GND #PWR04
U 1 1 5BBEBDF4
P 1700 1500
F 0 "#PWR04" H 1700 1250 50  0001 C CNN
F 1 "GND" H 1705 1327 50  0000 C CNN
F 2 "" H 1700 1500 50  0001 C CNN
F 3 "" H 1700 1500 50  0001 C CNN
	1    1700 1500
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C1
U 1 1 5BBEC4B8
P 1000 1150
F 0 "C1" H 1178 1196 50  0000 L CNN
F 1 "1uF" H 1178 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1000 1150 50  0001 C CNN
F 3 "" H 1000 1150 50  0001 C CNN
	1    1000 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  900  1000 900 
Wire Wire Line
	1000 900  1250 900 
Connection ~ 1000 900 
Connection ~ 1250 900 
$Comp
L power:GND #PWR03
U 1 1 5BBEC681
P 1000 1500
F 0 "#PWR03" H 1000 1250 50  0001 C CNN
F 1 "GND" H 1005 1327 50  0000 C CNN
F 2 "" H 1000 1500 50  0001 C CNN
F 3 "" H 1000 1500 50  0001 C CNN
	1    1000 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 1400 1000 1500
Wire Wire Line
	1700 1300 1700 1500
$Comp
L power:+3V3 #PWR05
U 1 1 5BBEC794
P 2250 900
F 0 "#PWR05" H 2250 750 50  0001 C CNN
F 1 "+3V3" H 2265 1073 50  0000 C CNN
F 2 "" H 2250 900 50  0001 C CNN
F 3 "" H 2250 900 50  0001 C CNN
	1    2250 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 900  2250 900 
$Comp
L pspice:CAP C2
U 1 1 5BBECE27
P 2400 1250
F 0 "C2" H 2578 1296 50  0000 L CNN
F 1 "10nF" H 2578 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2400 1250 50  0001 C CNN
F 3 "" H 2400 1250 50  0001 C CNN
	1    2400 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5BBECEE3
P 2400 1550
F 0 "#PWR06" H 2400 1300 50  0001 C CNN
F 1 "GND" H 2405 1377 50  0000 C CNN
F 2 "" H 2400 1550 50  0001 C CNN
F 3 "" H 2400 1550 50  0001 C CNN
	1    2400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1000 2400 1000
Wire Wire Line
	2400 1500 2400 1550
$Comp
L power:+3V3 #PWR027
U 1 1 5BBED3CA
P 9150 750
F 0 "#PWR027" H 9150 600 50  0001 C CNN
F 1 "+3V3" H 9165 923 50  0000 C CNN
F 2 "" H 9150 750 50  0001 C CNN
F 3 "" H 9150 750 50  0001 C CNN
	1    9150 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 900  9000 750 
Wire Wire Line
	9000 750  9100 750 
Wire Wire Line
	9100 900  9100 750 
Connection ~ 9100 750 
Wire Wire Line
	9100 750  9150 750 
Wire Wire Line
	9200 900  9200 750 
Wire Wire Line
	9200 750  9150 750 
Connection ~ 9150 750 
Wire Wire Line
	9300 900  9300 750 
Wire Wire Line
	9300 750  9200 750 
Connection ~ 9200 750 
Wire Wire Line
	9400 900  9400 750 
Wire Wire Line
	9400 750  9300 750 
Connection ~ 9300 750 
$Comp
L Device:C C4
U 1 1 5BBEFFAA
P 3650 1850
F 0 "C4" H 3765 1896 50  0000 L CNN
F 1 "0.1uF" H 3765 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3688 1700 50  0001 C CNN
F 3 "~" H 3650 1850 50  0001 C CNN
	1    3650 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5BBEFFB1
P 4000 1850
F 0 "C5" H 4115 1896 50  0000 L CNN
F 1 "10nF" H 4115 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4038 1700 50  0001 C CNN
F 3 "~" H 4000 1850 50  0001 C CNN
	1    4000 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5BBEFFBF
P 3700 2050
F 0 "#PWR011" H 3700 1800 50  0001 C CNN
F 1 "GND" H 3705 1877 50  0000 C CNN
F 2 "" H 3700 2050 50  0001 C CNN
F 3 "" H 3700 2050 50  0001 C CNN
	1    3700 2050
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR010
U 1 1 5BBEFFC5
P 3700 1650
F 0 "#PWR010" H 3700 1500 50  0001 C CNN
F 1 "+3V3" H 3715 1823 50  0000 C CNN
F 2 "" H 3700 1650 50  0001 C CNN
F 3 "" H 3700 1650 50  0001 C CNN
	1    3700 1650
	1    0    0    -1  
$EndComp
Connection ~ 3700 1650
Wire Wire Line
	4000 1700 4000 1650
Wire Wire Line
	4000 1650 3700 1650
Wire Wire Line
	3650 1700 3650 1650
Connection ~ 3650 1650
Wire Wire Line
	3650 1650 3700 1650
Connection ~ 3700 2050
Wire Wire Line
	4000 2000 4000 2050
Wire Wire Line
	4000 2050 3700 2050
Wire Wire Line
	3650 2000 3650 2050
Connection ~ 3650 2050
Wire Wire Line
	3650 2050 3700 2050
Wire Wire Line
	8900 4500 8900 4600
Wire Wire Line
	8900 4600 9000 4600
Wire Wire Line
	9000 4500 9000 4600
Connection ~ 9000 4600
Wire Wire Line
	9000 4600 9100 4600
Wire Wire Line
	9100 4500 9100 4600
Connection ~ 9100 4600
Wire Wire Line
	9100 4600 9200 4600
Wire Wire Line
	9200 4500 9200 4600
Connection ~ 9200 4600
Wire Wire Line
	9200 4600 9300 4600
Wire Wire Line
	9300 4500 9300 4600
$Comp
L Device:C C6
U 1 1 5BC062E6
P 7600 1250
F 0 "C6" H 7715 1296 50  0000 L CNN
F 1 "0.1uF" H 7715 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7638 1100 50  0001 C CNN
F 3 "~" H 7600 1250 50  0001 C CNN
	1    7600 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5BC06352
P 7600 1850
F 0 "#PWR023" H 7600 1600 50  0001 C CNN
F 1 "GND" H 7605 1677 50  0000 C CNN
F 2 "" H 7600 1850 50  0001 C CNN
F 3 "" H 7600 1850 50  0001 C CNN
	1    7600 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1400 7600 1850
$Comp
L Device:C C7
U 1 1 5BC08697
P 8150 1650
F 0 "C7" H 8265 1696 50  0000 L CNN
F 1 "2.2uF" H 8265 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8188 1500 50  0001 C CNN
F 3 "~" H 8150 1650 50  0001 C CNN
	1    8150 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5BC086F7
P 8150 1850
F 0 "#PWR025" H 8150 1600 50  0001 C CNN
F 1 "GND" H 8155 1677 50  0000 C CNN
F 2 "" H 8150 1850 50  0001 C CNN
F 3 "" H 8150 1850 50  0001 C CNN
	1    8150 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 1500 8400 1500
Wire Wire Line
	8150 1800 8150 1850
Text GLabel 9950 2400 2    50   Input ~ 0
SWDIO
Wire Wire Line
	9800 2400 9950 2400
Text GLabel 9950 2500 2    50   Input ~ 0
SWCLK
Wire Wire Line
	9950 2500 9800 2500
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5BC0DC52
P 3400 900
F 0 "J2" H 3506 1178 50  0000 C CNN
F 1 "Conn_01x04_Male" H 3506 1087 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3400 900 50  0001 C CNN
F 3 "~" H 3400 900 50  0001 C CNN
	1    3400 900 
	1    0    0    -1  
$EndComp
Text GLabel 3700 800  2    50   Input ~ 0
SWDIO
Text GLabel 3700 900  2    50   Input ~ 0
SWCLK
$Comp
L power:+3V3 #PWR08
U 1 1 5BC0DF70
P 3700 1000
F 0 "#PWR08" H 3700 850 50  0001 C CNN
F 1 "+3V3" V 3715 1128 50  0000 L CNN
F 2 "" H 3700 1000 50  0001 C CNN
F 3 "" H 3700 1000 50  0001 C CNN
	1    3700 1000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5BC0E02F
P 3700 1100
F 0 "#PWR09" H 3700 850 50  0001 C CNN
F 1 "GND" H 3705 927 50  0000 C CNN
F 2 "" H 3700 1100 50  0001 C CNN
F 3 "" H 3700 1100 50  0001 C CNN
	1    3700 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1100 3600 1100
Wire Wire Line
	3600 1000 3700 1000
Wire Wire Line
	3600 900  3700 900 
Wire Wire Line
	3600 800  3700 800 
Text GLabel 9850 1100 2    50   Input ~ 0
TIM5_CH1
Text GLabel 9850 1200 2    50   Input ~ 0
TIM5_CH2
Text GLabel 9850 1300 2    50   Input ~ 0
TIM5_CH3
Text GLabel 9850 1400 2    50   Input ~ 0
TIM5_CH4
Wire Wire Line
	9800 1100 9850 1100
Wire Wire Line
	9800 1200 9850 1200
Wire Wire Line
	9850 1300 9800 1300
Wire Wire Line
	9800 1400 9850 1400
$Comp
L power:GND #PWR026
U 1 1 5BC42F7D
P 9100 4750
F 0 "#PWR026" H 9100 4500 50  0001 C CNN
F 1 "GND" H 9105 4577 50  0000 C CNN
F 2 "" H 9100 4750 50  0001 C CNN
F 3 "" H 9100 4750 50  0001 C CNN
	1    9100 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 4750 9100 4600
Wire Wire Line
	9000 750  8900 750 
Wire Wire Line
	8900 750  8900 900 
Connection ~ 9000 750 
Wire Wire Line
	7600 1100 8400 1100
$Comp
L Device:R_US R7
U 1 1 5BC56AA8
P 7900 1450
F 0 "R7" V 7695 1450 50  0000 C CNN
F 1 "10k" V 7786 1450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7940 1440 50  0001 C CNN
F 3 "~" H 7900 1450 50  0001 C CNN
	1    7900 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	8400 1300 7900 1300
$Comp
L power:GND #PWR024
U 1 1 5BC59AFA
P 7900 1850
F 0 "#PWR024" H 7900 1600 50  0001 C CNN
F 1 "GND" H 7905 1677 50  0000 C CNN
F 2 "" H 7900 1850 50  0001 C CNN
F 3 "" H 7900 1850 50  0001 C CNN
	1    7900 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1600 7900 1850
Text GLabel 8350 3000 0    50   Input ~ 0
SPI2_MISO
Text GLabel 8350 3100 0    50   Input ~ 0
SPI2_MOSI
Text GLabel 8350 3500 0    50   Input ~ 0
SPI2_CLK
Text GLabel 9850 3100 2    50   Input ~ 0
SPI1_CLK
Text GLabel 9850 3200 2    50   Input ~ 0
SPI1_MISO
Text GLabel 9850 3300 2    50   Input ~ 0
SPI1_MOSI
Wire Wire Line
	8400 3000 8350 3000
Wire Wire Line
	8350 3100 8400 3100
Wire Wire Line
	8350 3500 8400 3500
Wire Wire Line
	9800 3100 9850 3100
Wire Wire Line
	9800 3200 9850 3200
Wire Wire Line
	9800 3300 9850 3300
Text GLabel 8350 2800 0    50   Input ~ 0
SPI2_CS0
Text GLabel 8350 2900 0    50   Input ~ 0
SPI2_CS1
Wire Wire Line
	8350 2900 8400 2900
Wire Wire Line
	8350 2800 8400 2800
Wire Notes Line
	3100 550  3100 1350
Wire Notes Line
	3100 1350 4100 1350
Wire Notes Line
	4100 1350 4100 550 
Wire Notes Line
	4100 550  3100 550 
Wire Notes Line
	3100 1400 3100 2300
Wire Notes Line
	3100 2300 4300 2300
Wire Notes Line
	4300 2300 4300 1400
Text GLabel 9850 1900 2    50   Input ~ 0
TIM1_CH1
Text GLabel 9850 2000 2    50   Input ~ 0
TIM1_CH2
Text GLabel 9850 2100 2    50   Input ~ 0
TIM1_CH3
Text GLabel 9850 2200 2    50   Input ~ 0
TIM1_CH4
Wire Wire Line
	9850 1900 9800 1900
Wire Wire Line
	9800 2000 9850 2000
Wire Wire Line
	9850 2100 9800 2100
Wire Wire Line
	9800 2200 9850 2200
Text GLabel 2150 3450 0    50   Input ~ 0
OUTA
Text GLabel 2150 3550 0    50   Input ~ 0
OUTB
Text GLabel 2250 3450 2    50   Input ~ 0
TIM5_CH1
Text GLabel 2250 3550 2    50   Input ~ 0
TIM5_CH2
Wire Wire Line
	2250 3450 2150 3450
Wire Wire Line
	2150 3550 2250 3550
Text GLabel 8350 3200 0    50   Input ~ 0
SPI2_CS2
Wire Wire Line
	8350 3200 8400 3200
Text GLabel 9850 3400 2    50   Input ~ 0
USART1_TX
Text GLabel 9850 3500 2    50   Input ~ 0
USART1_RX
Wire Wire Line
	9800 3400 9850 3400
Wire Wire Line
	9800 3500 9850 3500
Text GLabel 8350 4300 0    50   Input ~ 0
LED_D
Text GLabel 8350 4200 0    50   Input ~ 0
LED_C
Text GLabel 8350 4100 0    50   Input ~ 0
LED_B
Text GLabel 8350 4000 0    50   Input ~ 0
LED_A
Wire Wire Line
	8350 4300 8400 4300
Wire Wire Line
	8350 4200 8400 4200
Wire Wire Line
	8350 4100 8400 4100
Wire Wire Line
	8350 4000 8400 4000
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 5C908AD8
P 1300 2250
F 0 "J1" H 1327 2226 50  0000 L CNN
F 1 "Conn_01x02_Female" H 1327 2135 50  0000 L CNN
F 2 "Connector_PinSocket_2.00mm:PinSocket_1x02_P2.00mm_Vertical" H 1300 2250 50  0001 C CNN
F 3 "~" H 1300 2250 50  0001 C CNN
	1    1300 2250
	1    0    0    -1  
$EndComp
Text GLabel 1050 2250 0    50   Input ~ 0
USART1_TX
Text GLabel 1050 2350 0    50   Input ~ 0
USART1_RX
Wire Wire Line
	1050 2250 1100 2250
Wire Wire Line
	1050 2350 1100 2350
Text GLabel 9850 2600 2    50   Input ~ 0
SPI1_CS0
Wire Wire Line
	9850 2600 9800 2600
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 5C5B1F64
P 4900 650
F 0 "J5" H 4927 676 50  0000 L CNN
F 1 "Conn_01x01_Female" H 4927 585 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_3.0x3.0mm" H 4900 650 50  0001 C CNN
F 3 "~" H 4900 650 50  0001 C CNN
	1    4900 650 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 5C5B1FE6
P 4900 800
F 0 "J6" H 4927 826 50  0000 L CNN
F 1 "Conn_01x01_Female" H 4927 735 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_3.0x3.0mm" H 4900 800 50  0001 C CNN
F 3 "~" H 4900 800 50  0001 C CNN
	1    4900 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5C5B2194
P 4650 650
F 0 "#PWR012" H 4650 400 50  0001 C CNN
F 1 "GND" V 4655 477 50  0000 C CNN
F 2 "" H 4650 650 50  0001 C CNN
F 3 "" H 4650 650 50  0001 C CNN
	1    4650 650 
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 650  4700 650 
Wire Notes Line
	4300 1400 3100 1400
$Comp
L kyleRhess_Misc:HCMS-2903 U2
U 1 1 5F34A079
P 1700 4700
F 0 "U2" H 1700 5215 50  0000 C CNN
F 1 "HCMS-2903" H 1700 5124 50  0000 C CNN
F 2 "kyleRhess_packages:HCMS-290x" H 2000 4300 50  0001 C CNN
F 3 "" H 2000 4300 50  0001 C CNN
	1    1700 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 5F34C748
P 2550 4650
F 0 "R5" V 2345 4650 50  0000 C CNN
F 1 "10k" V 2436 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2590 4640 50  0001 C CNN
F 3 "~" H 2550 4650 50  0001 C CNN
	1    2550 4650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 5BBEC55E
P 650 900
F 0 "#PWR02" H 650 750 50  0001 C CNN
F 1 "+5V" H 665 1073 50  0000 C CNN
F 2 "" H 650 900 50  0001 C CNN
F 3 "" H 650 900 50  0001 C CNN
	1    650  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4650 2200 4650
$Comp
L power:GND #PWR021
U 1 1 5F35B499
P 2850 4900
F 0 "#PWR021" H 2850 4650 50  0001 C CNN
F 1 "GND" H 2855 4727 50  0000 C CNN
F 2 "" H 2850 4900 50  0001 C CNN
F 3 "" H 2850 4900 50  0001 C CNN
	1    2850 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 4750 2850 4750
Wire Wire Line
	2850 4750 2850 4900
$Comp
L power:+5V #PWR07
U 1 1 5F368BEE
P 950 4650
F 0 "#PWR07" H 950 4500 50  0001 C CNN
F 1 "+5V" H 965 4823 50  0000 C CNN
F 2 "" H 950 4650 50  0001 C CNN
F 3 "" H 950 4650 50  0001 C CNN
	1    950  4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4650 2850 4650
Wire Wire Line
	2850 4650 2850 4550
Wire Wire Line
	2200 4550 2850 4550
Connection ~ 2850 4550
Wire Wire Line
	2850 4550 2850 4400
Wire Wire Line
	1200 4650 950  4650
$Comp
L Switch:SW_Push SW1
U 1 1 5F38009B
P 7350 1100
F 0 "SW1" H 7350 1385 50  0000 C CNN
F 1 "SW_Push" H 7350 1294 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_CK_KMR2" H 7350 1300 50  0001 C CNN
F 3 "" H 7350 1300 50  0001 C CNN
	1    7350 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R6
U 1 1 5F380C36
P 7150 1400
F 0 "R6" V 6945 1400 50  0000 C CNN
F 1 "470" V 7036 1400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7190 1390 50  0001 C CNN
F 3 "~" H 7150 1400 50  0001 C CNN
	1    7150 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	7150 1250 7150 1100
$Comp
L power:GND #PWR022
U 1 1 5F38D1B6
P 7150 1850
F 0 "#PWR022" H 7150 1600 50  0001 C CNN
F 1 "GND" H 7155 1677 50  0000 C CNN
F 2 "" H 7150 1850 50  0001 C CNN
F 3 "" H 7150 1850 50  0001 C CNN
	1    7150 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1550 7150 1850
Wire Notes Line
	550  2150 2100 2150
Wire Notes Line
	2100 2150 2100 2450
Wire Notes Line
	2100 2450 550  2450
Wire Notes Line
	550  2450 550  2150
Wire Wire Line
	7550 1100 7600 1100
Connection ~ 7600 1100
Text GLabel 1500 2750 2    50   Input ~ 0
LED_A
Text GLabel 1500 3050 2    50   Input ~ 0
LED_B
Text GLabel 1500 3350 2    50   Input ~ 0
LED_C
Text GLabel 1500 3650 2    50   Input ~ 0
LED_D
Wire Wire Line
	650  3050 650  3350
Connection ~ 650  3050
Wire Wire Line
	750  3050 650  3050
Wire Wire Line
	650  3350 650  3650
Connection ~ 650  3350
Wire Wire Line
	750  3350 650  3350
Wire Wire Line
	650  3650 650  3750
Connection ~ 650  3650
Wire Wire Line
	650  3650 750  3650
Wire Wire Line
	650  2750 650  3050
Wire Wire Line
	750  2750 650  2750
$Comp
L power:GND #PWR01
U 1 1 5BC4920F
P 650 3750
F 0 "#PWR01" H 650 3500 50  0001 C CNN
F 1 "GND" H 655 3577 50  0000 C CNN
F 2 "" H 650 3750 50  0001 C CNN
F 3 "" H 650 3750 50  0001 C CNN
	1    650  3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2750 1450 2750
Wire Wire Line
	1500 3050 1450 3050
Wire Wire Line
	1500 3350 1450 3350
Wire Wire Line
	1500 3650 1450 3650
Wire Wire Line
	1150 3650 1050 3650
$Comp
L Device:R_US R4
U 1 1 5BC2A0FD
P 1300 3650
F 0 "R4" V 1095 3650 50  0000 C CNN
F 1 "470" V 1186 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1340 3640 50  0001 C CNN
F 3 "~" H 1300 3650 50  0001 C CNN
	1    1300 3650
	0    1    1    0   
$EndComp
$Comp
L Device:LED D4
U 1 1 5BC2A0F6
P 900 3650
F 0 "D4" H 891 3866 50  0000 C CNN
F 1 "LED" H 891 3775 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 900 3650 50  0001 C CNN
F 3 "~" H 900 3650 50  0001 C CNN
	1    900  3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3350 1050 3350
$Comp
L Device:R_US R3
U 1 1 5BC286CF
P 1300 3350
F 0 "R3" V 1095 3350 50  0000 C CNN
F 1 "470" V 1186 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1340 3340 50  0001 C CNN
F 3 "~" H 1300 3350 50  0001 C CNN
	1    1300 3350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5BC286C8
P 900 3350
F 0 "D3" H 891 3566 50  0000 C CNN
F 1 "LED" H 891 3475 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 900 3350 50  0001 C CNN
F 3 "~" H 900 3350 50  0001 C CNN
	1    900  3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3050 1050 3050
$Comp
L Device:R_US R2
U 1 1 5BC26D85
P 1300 3050
F 0 "R2" V 1095 3050 50  0000 C CNN
F 1 "470" V 1186 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1340 3040 50  0001 C CNN
F 3 "~" H 1300 3050 50  0001 C CNN
	1    1300 3050
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5BC26D7E
P 900 3050
F 0 "D2" H 891 3266 50  0000 C CNN
F 1 "LED" H 891 3175 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 900 3050 50  0001 C CNN
F 3 "~" H 900 3050 50  0001 C CNN
	1    900  3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2750 1050 2750
$Comp
L Device:R_US R1
U 1 1 5BC1BB56
P 1300 2750
F 0 "R1" V 1095 2750 50  0000 C CNN
F 1 "470" V 1186 2750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1340 2740 50  0001 C CNN
F 3 "~" H 1300 2750 50  0001 C CNN
	1    1300 2750
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5BC188F9
P 900 2750
F 0 "D1" H 891 2966 50  0000 C CNN
F 1 "LED" H 891 2875 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 900 2750 50  0001 C CNN
F 3 "~" H 900 2750 50  0001 C CNN
	1    900  2750
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  4000 1800 4000
Wire Notes Line
	1800 4000 1800 2500
Wire Notes Line
	1800 2500 550  2500
Wire Notes Line
	550  2500 550  4000
Text GLabel 1100 4950 0    50   Input ~ 0
SPI1_CLK
Wire Wire Line
	1200 4950 1100 4950
Text GLabel 9850 2800 2    50   Input ~ 0
BLANK
Wire Wire Line
	9850 2800 9800 2800
Text GLabel 2350 4850 2    50   Input ~ 0
BLANK
Wire Wire Line
	2350 4850 2200 4850
$Comp
L power:+3V3 #PWR020
U 1 1 5F43CA2C
P 2850 4400
F 0 "#PWR020" H 2850 4250 50  0001 C CNN
F 1 "+3V3" H 2865 4573 50  0000 C CNN
F 2 "" H 2850 4400 50  0001 C CNN
F 3 "" H 2850 4400 50  0001 C CNN
	1    2850 4400
	1    0    0    -1  
$EndComp
Text GLabel 9850 2900 2    50   Input ~ 0
LRESET
Wire Wire Line
	9850 2900 9800 2900
Text GLabel 2350 4350 2    50   Input ~ 0
LRESET
Wire Wire Line
	2350 4350 2250 4350
Wire Wire Line
	2250 4350 2250 4450
Wire Wire Line
	2250 4450 2200 4450
Text GLabel 2350 4950 2    50   Input ~ 0
SPI1_CS0
Wire Wire Line
	2350 4950 2200 4950
Text GLabel 1100 4750 0    50   Input ~ 0
SPI1_MOSI
Wire Wire Line
	1100 4750 1200 4750
NoConn ~ 1200 4550
NoConn ~ 1200 4450
Text GLabel 9850 3600 2    50   Input ~ 0
RS
Wire Wire Line
	9850 3600 9800 3600
Text GLabel 1100 4850 0    50   Input ~ 0
RS
Wire Wire Line
	1100 4850 1200 4850
Wire Notes Line
	2800 550  2800 2100
Text GLabel 2400 2750 2    50   Input ~ 0
OUTA
Text GLabel 2400 2850 2    50   Input ~ 0
OUTB
Text GLabel 9850 1800 2    50   Input ~ 0
OUTZ
Wire Wire Line
	9850 1800 9800 1800
Text GLabel 2400 2950 2    50   Input ~ 0
OUTZ
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 5F5C18BA
P 2100 2950
F 0 "J3" H 2208 3331 50  0000 C CNN
F 1 "Conn_01x05_Male" H 2208 3240 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x05_P2.00mm_Vertical" H 2100 2950 50  0001 C CNN
F 3 "~" H 2100 2950 50  0001 C CNN
	1    2100 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5F5C8A55
P 2400 3150
F 0 "#PWR028" H 2400 2900 50  0001 C CNN
F 1 "GND" H 2405 2977 50  0000 C CNN
F 2 "" H 2400 3150 50  0001 C CNN
F 3 "" H 2400 3150 50  0001 C CNN
	1    2400 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR029
U 1 1 5F5C8E7F
P 2700 3050
F 0 "#PWR029" H 2700 2900 50  0001 C CNN
F 1 "+5V" H 2715 3223 50  0000 C CNN
F 2 "" H 2700 3050 50  0001 C CNN
F 3 "" H 2700 3050 50  0001 C CNN
	1    2700 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3050 2300 3050
Wire Wire Line
	2300 3150 2400 3150
Wire Wire Line
	2300 2950 2400 2950
Wire Wire Line
	2300 2850 2400 2850
Wire Wire Line
	2300 2750 2400 2750
Wire Notes Line
	1850 2500 2850 2500
Wire Notes Line
	2850 2500 2850 3750
Wire Notes Line
	2850 3750 1850 3750
Wire Notes Line
	4350 1850 4350 550 
Wire Notes Line
	4350 550  5700 550 
Wire Notes Line
	5700 550  5700 1850
Wire Notes Line
	5700 1850 4350 1850
$Comp
L power:+5V #PWR013
U 1 1 5F68B839
P 4600 800
F 0 "#PWR013" H 4600 650 50  0001 C CNN
F 1 "+5V" H 4615 973 50  0000 C CNN
F 2 "" H 4600 800 50  0001 C CNN
F 3 "" H 4600 800 50  0001 C CNN
	1    4600 800 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4600 800  4700 800 
Wire Notes Line
	650  5150 3000 5150
Wire Notes Line
	3000 5150 3000 4150
Wire Notes Line
	3000 4150 650  4150
Wire Notes Line
	650  4150 650  5150
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 5F327162
P 4900 950
F 0 "J4" H 4927 976 50  0000 L CNN
F 1 "Conn_01x01_Female" H 4927 885 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_3.0x3.0mm" H 4900 950 50  0001 C CNN
F 3 "~" H 4900 950 50  0001 C CNN
	1    4900 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2050 3650 2050
Wire Wire Line
	3250 1650 3650 1650
Wire Wire Line
	3250 2000 3250 2050
Wire Wire Line
	3250 1700 3250 1650
$Comp
L Device:C C3
U 1 1 5BBEFFA3
P 3250 1850
F 0 "C3" H 3365 1896 50  0000 L CNN
F 1 "2.2uF" H 3365 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3288 1700 50  0001 C CNN
F 3 "~" H 3250 1850 50  0001 C CNN
	1    3250 1850
	1    0    0    -1  
$EndComp
Text GLabel 4650 950  0    50   Input ~ 0
PWM1
Wire Wire Line
	4650 950  4700 950 
Text GLabel 2150 3650 0    50   Input ~ 0
PWM1
Text GLabel 2250 3650 2    50   Input ~ 0
TIM1_CH1
Wire Wire Line
	2150 3650 2250 3650
Wire Notes Line
	1850 3750 1850 2500
Text GLabel 9850 1600 2    50   Input ~ 0
DAC_1
Wire Wire Line
	9850 1600 9800 1600
Text GLabel 4650 1100 0    50   Input ~ 0
DAC_1
$Comp
L Connector:Conn_01x01_Female J7
U 1 1 5F3064C5
P 4900 1100
F 0 "J7" H 4927 1126 50  0000 L CNN
F 1 "Conn_01x01_Female" H 4927 1035 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D2.0mm" H 4900 1100 50  0001 C CNN
F 3 "~" H 4900 1100 50  0001 C CNN
	1    4900 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 1100 4650 1100
$EndSCHEMATC
