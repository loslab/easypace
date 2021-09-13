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
L Amplifier_Operational:LM324 U3
U 1 1 603CDCC8
P 5100 3300
F 0 "U3" H 4950 2950 50  0000 C CNN
F 1 "LM324" H 4950 3050 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5050 3400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5150 3500 50  0001 C CNN
	1    5100 3300
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:LM324 U3
U 2 1 603D189E
P 5100 4400
F 0 "U3" H 5250 4150 50  0000 C CNN
F 1 "LM324" H 5250 4250 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5050 4500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5150 4600 50  0001 C CNN
	2    5100 4400
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:LM324 U3
U 3 1 603D2E11
P 5100 5600
F 0 "U3" H 5250 5350 50  0000 C CNN
F 1 "LM324" H 5250 5450 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5050 5700 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5150 5800 50  0001 C CNN
	3    5100 5600
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:LM324 U3
U 4 1 603D421E
P 5100 6650
F 0 "U3" H 5250 6400 50  0000 C CNN
F 1 "LM324" H 5250 6500 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5050 6750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5150 6850 50  0001 C CNN
	4    5100 6650
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:LM324 U3
U 5 1 603D541E
P 5300 3300
F 0 "U3" H 5300 3650 50  0000 L CNN
F 1 "LM324" H 5250 3550 50  0000 L CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5250 3400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5350 3500 50  0001 C CNN
	5    5300 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 603DC244
P 4500 3200
F 0 "R9" V 4293 3200 50  0000 C CNN
F 1 "11k" V 4384 3200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 3200 50  0001 C CNN
F 3 "~" H 4500 3200 50  0001 C CNN
	1    4500 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 603DCEC6
P 5100 2700
F 0 "R17" V 4893 2700 50  0000 C CNN
F 1 "56k" V 4984 2700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5030 2700 50  0001 C CNN
F 3 "~" H 5100 2700 50  0001 C CNN
	1    5100 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 603DD3CF
P 4250 3400
F 0 "R5" V 4043 3400 50  0000 C CNN
F 1 "11k" V 4134 3400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4180 3400 50  0001 C CNN
F 3 "~" H 4250 3400 50  0001 C CNN
	1    4250 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 603DD7BB
P 4500 3550
F 0 "R10" H 4430 3504 50  0000 R CNN
F 1 "56k" H 4430 3595 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 3550 50  0001 C CNN
F 3 "~" H 4500 3550 50  0001 C CNN
	1    4500 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 3200 4700 3200
Wire Wire Line
	4700 3200 4700 2700
Wire Wire Line
	4700 2700 4950 2700
Connection ~ 4700 3200
Wire Wire Line
	4700 3200 4800 3200
Wire Wire Line
	5250 2700 5450 2700
Wire Wire Line
	5450 2700 5450 3300
Wire Wire Line
	5450 3300 5400 3300
Wire Wire Line
	4400 3400 4500 3400
Wire Wire Line
	4500 3400 4800 3400
Connection ~ 4500 3400
$Comp
L Reference_Voltage:LM285Z-2.5 U2
U 1 1 603DEDDF
P 3350 1050
F 0 "U2" H 3350 825 50  0000 C CNN
F 1 "LM285Z-2.5" H 3350 916 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3350 850 50  0001 C CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/LM285-D.PDF" H 3350 1050 50  0001 C CIN
	1    3350 1050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 603E0B46
P 3000 1050
F 0 "R4" V 2793 1050 50  0000 C CNN
F 1 "1k" V 2884 1050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2930 1050 50  0001 C CNN
F 3 "~" H 3000 1050 50  0001 C CNN
	1    3000 1050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 603E0FCB
P 3600 1050
F 0 "#PWR013" H 3600 800 50  0001 C CNN
F 1 "GND" H 3605 877 50  0000 C CNN
F 2 "" H 3600 1050 50  0001 C CNN
F 3 "" H 3600 1050 50  0001 C CNN
	1    3600 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 603E1BE4
P 2750 6850
F 0 "#PWR010" H 2750 6600 50  0001 C CNN
F 1 "GND" H 2755 6677 50  0000 C CNN
F 2 "" H 2750 6850 50  0001 C CNN
F 3 "" H 2750 6850 50  0001 C CNN
	1    2750 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 603E1E71
P 2100 6750
F 0 "R3" V 1893 6750 50  0000 C CNN
F 1 "220" V 1984 6750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 6750 50  0001 C CNN
F 3 "~" H 2100 6750 50  0001 C CNN
	1    2100 6750
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 603E23C5
P 2100 6350
F 0 "R2" V 1893 6350 50  0000 C CNN
F 1 "220" V 1984 6350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2030 6350 50  0001 C CNN
F 3 "~" H 2100 6350 50  0001 C CNN
	1    2100 6350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 603E2676
P 2450 6750
F 0 "D2" H 2443 6495 50  0000 C CNN
F 1 "LED" H 2443 6586 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 2450 6750 50  0001 C CNN
F 3 "~" H 2450 6750 50  0001 C CNN
	1    2450 6750
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D1
U 1 1 603E34EB
P 2450 6350
F 0 "D1" H 2443 6095 50  0000 C CNN
F 1 "LED" H 2443 6186 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 2450 6350 50  0001 C CNN
F 3 "~" H 2450 6350 50  0001 C CNN
	1    2450 6350
	-1   0    0    1   
$EndComp
Wire Wire Line
	2250 6350 2300 6350
Wire Wire Line
	2250 6750 2300 6750
Wire Wire Line
	2600 6750 2750 6750
Wire Wire Line
	2750 6750 2750 6350
Wire Wire Line
	2750 6350 2600 6350
Wire Wire Line
	2750 6850 2750 6750
Connection ~ 2750 6750
$Comp
L Device:R R1
U 1 1 603E50B9
P 1300 5250
F 0 "R1" H 1230 5204 50  0000 R CNN
F 1 "10k" H 1230 5295 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1230 5250 50  0001 C CNN
F 3 "~" H 1300 5250 50  0001 C CNN
	1    1300 5250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 603E56B0
P 1300 5700
F 0 "#PWR03" H 1300 5450 50  0001 C CNN
F 1 "GND" H 1305 5527 50  0000 C CNN
F 2 "" H 1300 5700 50  0001 C CNN
F 3 "" H 1300 5700 50  0001 C CNN
	1    1300 5700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 603E5A42
P 2300 4650
F 0 "#PWR07" H 2300 4500 50  0001 C CNN
F 1 "+5V" H 2315 4823 50  0000 C CNN
F 2 "" H 2300 4650 50  0001 C CNN
F 3 "" H 2300 4650 50  0001 C CNN
	1    2300 4650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR06
U 1 1 603E6C09
P 1800 1850
F 0 "#PWR06" H 1800 1700 50  0001 C CNN
F 1 "+5V" H 1815 2023 50  0000 C CNN
F 2 "" H 1800 1850 50  0001 C CNN
F 3 "" H 1800 1850 50  0001 C CNN
	1    1800 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 603E6F91
P 1650 4000
F 0 "#PWR05" H 1650 3750 50  0001 C CNN
F 1 "GND" H 1655 3827 50  0000 C CNN
F 2 "" H 1650 4000 50  0001 C CNN
F 3 "" H 1650 4000 50  0001 C CNN
	1    1650 4000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR09
U 1 1 603E7235
P 2700 1050
F 0 "#PWR09" H 2700 900 50  0001 C CNN
F 1 "+5V" H 2715 1223 50  0000 C CNN
F 2 "" H 2700 1050 50  0001 C CNN
F 3 "" H 2700 1050 50  0001 C CNN
	1    2700 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1050 2850 1050
Wire Wire Line
	3150 1050 3200 1050
Wire Wire Line
	3500 1050 3600 1050
Wire Wire Line
	3200 1050 3200 1250
Connection ~ 3200 1050
Text Label 3200 1250 2    50   ~ 0
VREF_2
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 603E9A26
P 5150 1600
F 0 "J2" H 5068 1175 50  0000 C CNN
F 1 "Conn_Enc" H 5068 1266 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5150 1600 50  0001 C CNN
F 3 "~" H 5150 1600 50  0001 C CNN
	1    5150 1600
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 603EAB46
P 6250 1600
F 0 "J3" H 6168 1175 50  0000 C CNN
F 1 "Conn_Disp" H 6168 1266 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6250 1600 50  0001 C CNN
F 3 "~" H 6250 1600 50  0001 C CNN
	1    6250 1600
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J6
U 1 1 603EC329
P 7250 3050
F 0 "J6" H 7300 2725 50  0000 C CNN
F 1 "Conn_Sigout1" H 7300 2816 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 7250 3050 50  0001 C CNN
F 3 "~" H 7250 3050 50  0001 C CNN
	1    7250 3050
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J7
U 1 1 603ED80E
P 7250 3750
F 0 "J7" H 7300 3425 50  0000 C CNN
F 1 "Conn_Sigout2" H 7300 3516 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 7250 3750 50  0001 C CNN
F 3 "~" H 7250 3750 50  0001 C CNN
	1    7250 3750
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR01
U 1 1 603EE115
P 950 950
F 0 "#PWR01" H 950 800 50  0001 C CNN
F 1 "+12V" H 965 1123 50  0000 C CNN
F 2 "" H 950 950 50  0001 C CNN
F 3 "" H 950 950 50  0001 C CNN
	1    950  950 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR04
U 1 1 603EF0FD
P 1500 1850
F 0 "#PWR04" H 1500 1700 50  0001 C CNN
F 1 "+12V" H 1515 2023 50  0000 C CNN
F 2 "" H 1500 1850 50  0001 C CNN
F 3 "" H 1500 1850 50  0001 C CNN
	1    1500 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 603EF783
P 7900 2950
F 0 "#PWR024" H 7900 2700 50  0001 C CNN
F 1 "GND" H 7905 2777 50  0000 C CNN
F 2 "" H 7900 2950 50  0001 C CNN
F 3 "" H 7900 2950 50  0001 C CNN
	1    7900 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 603EFCA8
P 7900 3650
F 0 "#PWR025" H 7900 3400 50  0001 C CNN
F 1 "GND" H 7905 3477 50  0000 C CNN
F 2 "" H 7900 3650 50  0001 C CNN
F 3 "" H 7900 3650 50  0001 C CNN
	1    7900 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3650 7900 3650
Wire Wire Line
	7450 2950 7900 2950
$Comp
L Converter_DCDC:IA1215S PS1
U 1 1 603F2349
P 1500 1050
F 0 "PS1" H 1500 1417 50  0000 C CNN
F 1 "SIM2-1215D" H 1500 1326 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_XP_POWER-IAxxxxS_THT" H 450 800 50  0001 L CNN
F 3 "https://www.xppower.com/pdfs/SF_IA.pdf" H 2550 750 50  0001 L CNN
	1    1500 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 603F4996
P 950 1150
F 0 "#PWR02" H 950 900 50  0001 C CNN
F 1 "GND" H 955 977 50  0000 C CNN
F 2 "" H 950 1150 50  0001 C CNN
F 3 "" H 950 1150 50  0001 C CNN
	1    950  1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 603F4D39
P 2350 1050
F 0 "#PWR08" H 2350 800 50  0001 C CNN
F 1 "GND" H 2355 877 50  0000 C CNN
F 2 "" H 2350 1050 50  0001 C CNN
F 3 "" H 2350 1050 50  0001 C CNN
	1    2350 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  950  1100 950 
Wire Wire Line
	950  1150 1100 1150
Wire Wire Line
	1900 1050 2350 1050
Wire Wire Line
	1900 1150 1950 1150
Wire Wire Line
	1900 950  1950 950 
Text Label 1950 950  0    50   ~ 0
+VCC
Text Label 1950 1150 0    50   ~ 0
-VCC
$Comp
L power:GND #PWR021
U 1 1 603F8C20
P 5950 1700
F 0 "#PWR021" H 5950 1450 50  0001 C CNN
F 1 "GND" H 5955 1527 50  0000 C CNN
F 2 "" H 5950 1700 50  0001 C CNN
F 3 "" H 5950 1700 50  0001 C CNN
	1    5950 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 603F907F
P 4800 1800
F 0 "#PWR019" H 4800 1550 50  0001 C CNN
F 1 "GND" H 4805 1627 50  0000 C CNN
F 2 "" H 4800 1800 50  0001 C CNN
F 3 "" H 4800 1800 50  0001 C CNN
	1    4800 1800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR020
U 1 1 603F9339
P 5850 1600
F 0 "#PWR020" H 5850 1450 50  0001 C CNN
F 1 "+5V" V 5865 1728 50  0000 L CNN
F 2 "" H 5850 1600 50  0001 C CNN
F 3 "" H 5850 1600 50  0001 C CNN
	1    5850 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 603FA461
P 4700 1700
F 0 "#PWR018" H 4700 1550 50  0001 C CNN
F 1 "+5V" V 4715 1828 50  0000 L CNN
F 2 "" H 4700 1700 50  0001 C CNN
F 3 "" H 4700 1700 50  0001 C CNN
	1    4700 1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5950 1700 6050 1700
Wire Wire Line
	4800 1800 4950 1800
Wire Wire Line
	4700 1700 4950 1700
Wire Wire Line
	5900 1400 6050 1400
Wire Wire Line
	5900 1500 6050 1500
Text Label 5900 1400 0    50   ~ 0
SCL
Text Label 5900 1500 0    50   ~ 0
SDA
Wire Wire Line
	4650 1400 4950 1400
Wire Wire Line
	4650 1500 4950 1500
Wire Wire Line
	4650 1600 4950 1600
Text Label 4650 1400 0    50   ~ 0
ENC_CLK
Text Label 4650 1500 0    50   ~ 0
ENC_DT
Text Label 4650 1600 0    50   ~ 0
ENC_SW
Wire Wire Line
	5850 1600 6050 1600
Wire Wire Line
	1750 6350 1950 6350
Wire Wire Line
	1750 6750 1950 6750
Text Label 1750 6350 0    50   ~ 0
LED1
Text Label 1750 6750 0    50   ~ 0
LED2
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 60469A1A
P 1600 2950
F 0 "A1" H 1200 1950 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 1200 1850 50  0000 C CNN
F 2 "easypace_fps:Arduino_Nano" H 1600 2950 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 1600 2950 50  0001 C CNN
	1    1600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1950 1500 1850
Wire Wire Line
	1800 1950 1800 1850
NoConn ~ 1700 1950
Wire Wire Line
	1600 3950 1650 3950
Wire Wire Line
	1650 4000 1650 3950
Connection ~ 1650 3950
Wire Wire Line
	1650 3950 1700 3950
NoConn ~ 2100 2350
NoConn ~ 2100 2450
NoConn ~ 2100 2750
Wire Wire Line
	2100 3350 2250 3350
Wire Wire Line
	2100 3450 2250 3450
Text Label 2250 3350 0    50   ~ 0
SDA
Text Label 2250 3450 0    50   ~ 0
SCL
Text Label 1000 3650 2    50   ~ 0
CLK
Wire Wire Line
	1000 3650 1100 3650
Wire Wire Line
	1100 3350 1000 3350
Wire Wire Line
	1100 3450 1000 3450
Text Label 1000 3350 2    50   ~ 0
CS
Text Label 1000 3450 2    50   ~ 0
MOSI
Wire Wire Line
	1100 3150 1000 3150
Wire Wire Line
	1100 3050 1000 3050
Wire Wire Line
	1100 2950 1000 2950
Text Label 1000 3150 2    50   ~ 0
SHDN
Text Label 1000 3050 2    50   ~ 0
LED1
Text Label 1000 2950 2    50   ~ 0
LED2
Wire Wire Line
	1100 2750 1000 2750
Wire Wire Line
	1100 2650 1000 2650
Wire Wire Line
	1100 2550 1000 2550
Text Label 1000 2750 2    50   ~ 0
ENC_SW
Text Label 1000 2650 2    50   ~ 0
ENC_CLK
Text Label 1000 2550 2    50   ~ 0
ENC_DT
NoConn ~ 1100 2850
NoConn ~ 1100 3250
NoConn ~ 2100 3550
NoConn ~ 2100 3650
NoConn ~ 2100 3250
NoConn ~ 2100 3150
NoConn ~ 2100 3050
NoConn ~ 2100 2950
Wire Wire Line
	1800 5400 1700 5400
Wire Wire Line
	1800 5300 1700 5300
Text Label 1700 5300 2    50   ~ 0
MOSI
Text Label 1700 5400 2    50   ~ 0
CS
Wire Wire Line
	1800 5100 1700 5100
Text Label 1700 5100 2    50   ~ 0
CLK
Wire Wire Line
	1300 4900 1300 4800
Wire Wire Line
	1300 4900 1800 4900
Wire Wire Line
	1300 5100 1300 4900
Connection ~ 1300 4900
Text Label 1300 4800 2    50   ~ 0
SHDN
Wire Wire Line
	1300 5400 1300 5600
Connection ~ 1300 5600
Wire Wire Line
	1300 5600 1300 5700
Wire Wire Line
	1300 5600 1400 5600
Wire Wire Line
	1800 5200 1400 5200
Wire Wire Line
	1400 5200 1400 5600
Wire Wire Line
	2300 4700 2300 4650
Wire Wire Line
	2800 5000 2950 5000
Wire Wire Line
	2800 5300 2950 5300
Text Label 2950 5000 0    50   ~ 0
DAC1
Text Label 2950 5300 0    50   ~ 0
DAC2
Wire Wire Line
	2800 4900 2950 4900
Wire Wire Line
	2800 5400 2950 5400
Text Label 2950 4900 0    50   ~ 0
VREF_2
Text Label 2950 5400 0    50   ~ 0
VREF_2
Text Label 5200 2850 0    50   ~ 0
+VCC
Wire Wire Line
	5200 3600 5200 3700
Text Label 5200 3700 0    50   ~ 0
-VCC
Wire Wire Line
	5450 3300 5550 3300
Connection ~ 5450 3300
Text Label 5550 3300 0    50   ~ 0
SIG1
$Comp
L Device:R R18
U 1 1 604D7758
P 5100 4050
F 0 "R18" V 4893 4050 50  0000 C CNN
F 1 "56k" V 4984 4050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5030 4050 50  0001 C CNN
F 3 "~" H 5100 4050 50  0001 C CNN
	1    5100 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 604D7D69
P 4500 4300
F 0 "R11" V 4293 4300 50  0000 C CNN
F 1 "11k" V 4384 4300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 4300 50  0001 C CNN
F 3 "~" H 4500 4300 50  0001 C CNN
	1    4500 4300
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 604D8AF7
P 4250 4500
F 0 "R6" V 4043 4500 50  0000 C CNN
F 1 "11k" V 4134 4500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4180 4500 50  0001 C CNN
F 3 "~" H 4250 4500 50  0001 C CNN
	1    4250 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 604D8ED9
P 4500 4650
F 0 "R12" H 4430 4604 50  0000 R CNN
F 1 "56k" H 4430 4695 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 4650 50  0001 C CNN
F 3 "~" H 4500 4650 50  0001 C CNN
	1    4500 4650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 3400 4000 3400
Wire Wire Line
	4350 3200 4000 3200
Text Label 4000 3200 2    50   ~ 0
VREF_2
Text Label 4000 3400 2    50   ~ 0
DAC1
$Comp
L power:GND #PWR014
U 1 1 604DFD70
P 4500 3750
F 0 "#PWR014" H 4500 3500 50  0001 C CNN
F 1 "GND" H 4350 3700 50  0000 C CNN
F 2 "" H 4500 3750 50  0001 C CNN
F 3 "" H 4500 3750 50  0001 C CNN
	1    4500 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3750 4500 3700
$Comp
L Device:R R13
U 1 1 604F4D98
P 4500 5500
F 0 "R13" V 4293 5500 50  0000 C CNN
F 1 "11k" V 4384 5500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 5500 50  0001 C CNN
F 3 "~" H 4500 5500 50  0001 C CNN
	1    4500 5500
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 604F4D9E
P 4250 5700
F 0 "R7" V 4043 5700 50  0000 C CNN
F 1 "11k" V 4134 5700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4180 5700 50  0001 C CNN
F 3 "~" H 4250 5700 50  0001 C CNN
	1    4250 5700
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 604F4DA4
P 4500 5850
F 0 "R14" H 4430 5804 50  0000 R CNN
F 1 "56k" H 4430 5895 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 5850 50  0001 C CNN
F 3 "~" H 4500 5850 50  0001 C CNN
	1    4500 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 5500 4700 5500
Wire Wire Line
	4400 5700 4500 5700
Wire Wire Line
	4500 5700 4800 5700
Connection ~ 4500 5700
Wire Wire Line
	4100 5700 4000 5700
Wire Wire Line
	4350 5500 4000 5500
Text Label 4000 5500 2    50   ~ 0
VREF_2
Text Label 4000 5700 2    50   ~ 0
DAC2
$Comp
L power:GND #PWR016
U 1 1 604F4DB2
P 4500 6050
F 0 "#PWR016" H 4500 5800 50  0001 C CNN
F 1 "GND" H 4350 6000 50  0000 C CNN
F 2 "" H 4500 6050 50  0001 C CNN
F 3 "" H 4500 6050 50  0001 C CNN
	1    4500 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 6050 4500 6000
$Comp
L Device:R R15
U 1 1 604FA8B2
P 4500 6550
F 0 "R15" V 4293 6550 50  0000 C CNN
F 1 "11k" V 4384 6550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 6550 50  0001 C CNN
F 3 "~" H 4500 6550 50  0001 C CNN
	1    4500 6550
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 604FA8B8
P 4250 6750
F 0 "R8" V 4043 6750 50  0000 C CNN
F 1 "11k" V 4134 6750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4180 6750 50  0001 C CNN
F 3 "~" H 4250 6750 50  0001 C CNN
	1    4250 6750
	0    1    1    0   
$EndComp
$Comp
L Device:R R16
U 1 1 604FA8BE
P 4500 6900
F 0 "R16" H 4430 6854 50  0000 R CNN
F 1 "56k" H 4430 6945 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4430 6900 50  0001 C CNN
F 3 "~" H 4500 6900 50  0001 C CNN
	1    4500 6900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 6550 4700 6550
Wire Wire Line
	4400 6750 4500 6750
Wire Wire Line
	4500 6750 4800 6750
Connection ~ 4500 6750
Wire Wire Line
	4100 6750 4000 6750
Wire Wire Line
	4350 6550 4000 6550
Text Label 4000 6550 2    50   ~ 0
VREF_2
Text Label 4000 6750 2    50   ~ 0
DAC2
$Comp
L power:GND #PWR017
U 1 1 604FA8CC
P 4500 7100
F 0 "#PWR017" H 4500 6850 50  0001 C CNN
F 1 "GND" H 4350 7050 50  0000 C CNN
F 2 "" H 4500 7100 50  0001 C CNN
F 3 "" H 4500 7100 50  0001 C CNN
	1    4500 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 7100 4500 7050
$Comp
L Device:R R19
U 1 1 604FE9AD
P 5100 5200
F 0 "R19" V 4893 5200 50  0000 C CNN
F 1 "56k" V 4984 5200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5030 5200 50  0001 C CNN
F 3 "~" H 5100 5200 50  0001 C CNN
	1    5100 5200
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 604FF70D
P 5100 6300
F 0 "R20" V 4893 6300 50  0000 C CNN
F 1 "56k" V 4984 6300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5030 6300 50  0001 C CNN
F 3 "~" H 5100 6300 50  0001 C CNN
	1    5100 6300
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 5200 4700 5200
Wire Wire Line
	4700 5200 4700 5500
Wire Wire Line
	4800 5500 4700 5500
Connection ~ 4700 5500
Wire Wire Line
	5400 5600 5450 5600
Wire Wire Line
	5450 5600 5450 5200
Wire Wire Line
	5450 5200 5250 5200
Wire Wire Line
	4700 6550 4700 6300
Wire Wire Line
	4700 6300 4950 6300
Wire Wire Line
	4700 6550 4800 6550
Connection ~ 4700 6550
Wire Wire Line
	5250 6300 5450 6300
Wire Wire Line
	5450 6300 5450 6650
Wire Wire Line
	5450 6650 5400 6650
Wire Wire Line
	4400 4500 4500 4500
Wire Wire Line
	4500 4500 4800 4500
Connection ~ 4500 4500
Wire Wire Line
	4650 4300 4700 4300
Wire Wire Line
	4950 4050 4700 4050
Wire Wire Line
	4700 4050 4700 4300
Connection ~ 4700 4300
Wire Wire Line
	4700 4300 4800 4300
Wire Wire Line
	5250 4050 5450 4050
Wire Wire Line
	5450 4050 5450 4400
Wire Wire Line
	5450 4400 5400 4400
Wire Wire Line
	4100 4500 4000 4500
Wire Wire Line
	4000 4300 4350 4300
Wire Wire Line
	5450 4400 5550 4400
Connection ~ 5450 4400
Wire Wire Line
	5450 5600 5550 5600
Connection ~ 5450 5600
Wire Wire Line
	5450 6650 5550 6650
Connection ~ 5450 6650
Text Label 4000 4500 2    50   ~ 0
DAC1
Text Label 4000 4300 2    50   ~ 0
VREF_2
Text Label 5550 4400 0    50   ~ 0
SIG2
Text Label 5550 5600 0    50   ~ 0
SIG3
Text Label 5550 6650 0    50   ~ 0
SIG4
$Comp
L power:GND #PWR015
U 1 1 6057850C
P 4500 4900
F 0 "#PWR015" H 4500 4650 50  0001 C CNN
F 1 "GND" H 4350 4850 50  0000 C CNN
F 2 "" H 4500 4900 50  0001 C CNN
F 3 "" H 4500 4900 50  0001 C CNN
	1    4500 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4900 4500 4800
Wire Wire Line
	6950 3850 6850 3850
Wire Wire Line
	6950 3750 6850 3750
Wire Wire Line
	6950 3650 6850 3650
Text Label 6850 3650 2    50   ~ 0
SIG3
Text Label 6850 3750 2    50   ~ 0
SIG2
Text Label 6850 3850 2    50   ~ 0
SIG1
Wire Wire Line
	7450 3850 7600 3850
Text Label 7600 3850 0    50   ~ 0
SIG4
Wire Wire Line
	6950 3150 6850 3150
Wire Wire Line
	6950 3050 6850 3050
Wire Wire Line
	6950 2950 6850 2950
Text Label 6850 2950 2    50   ~ 0
SIG3
Text Label 6850 3050 2    50   ~ 0
SIG2
Text Label 6850 3150 2    50   ~ 0
SIG1
Text Label 7600 3150 0    50   ~ 0
SIG4
Wire Wire Line
	7450 3150 7600 3150
NoConn ~ 7450 3050
NoConn ~ 7450 3750
$Comp
L power:PWR_FLAG #FLG01
U 1 1 605B3202
P 3050 1750
F 0 "#FLG01" H 3050 1825 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 1923 50  0000 C CNN
F 2 "" H 3050 1750 50  0001 C CNN
F 3 "~" H 3050 1750 50  0001 C CNN
	1    3050 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 605B4A8B
P 2750 1850
F 0 "J1" H 2950 1750 50  0000 C CNN
F 1 "Conn_Vin" H 3000 1850 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 2750 1850 50  0001 C CNN
F 3 "~" H 2750 1850 50  0001 C CNN
	1    2750 1850
	-1   0    0    1   
$EndComp
$Comp
L power:+12V #PWR012
U 1 1 605B5D65
P 3400 1750
F 0 "#PWR012" H 3400 1600 50  0001 C CNN
F 1 "+12V" H 3415 1923 50  0000 C CNN
F 2 "" H 3400 1750 50  0001 C CNN
F 3 "" H 3400 1750 50  0001 C CNN
	1    3400 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1750 3050 1750
$Comp
L power:GND #PWR011
U 1 1 605BC324
P 3200 1850
F 0 "#PWR011" H 3200 1600 50  0001 C CNN
F 1 "GND" H 3205 1677 50  0000 C CNN
F 2 "" H 3200 1850 50  0001 C CNN
F 3 "" H 3200 1850 50  0001 C CNN
	1    3200 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1850 3200 1850
Connection ~ 3050 1750
Wire Wire Line
	3050 1750 3400 1750
Wire Wire Line
	6950 5300 6850 5300
Wire Wire Line
	6850 5200 6950 5200
Wire Wire Line
	6950 4750 6850 4750
Wire Wire Line
	6850 4650 6950 4650
Text Label 6850 4650 2    50   ~ 0
SIG4
Text Label 6850 4750 2    50   ~ 0
SIG3
Text Label 6850 5200 2    50   ~ 0
SIG2
Text Label 6850 5300 2    50   ~ 0
SIG1
Connection ~ 7450 5300
Wire Wire Line
	7450 5300 7600 5300
Wire Wire Line
	7450 5200 7450 5300
Connection ~ 7450 4750
Wire Wire Line
	7600 4750 7450 4750
Wire Wire Line
	7450 4650 7450 4750
$Comp
L power:GND #PWR023
U 1 1 605FBB51
P 7600 5300
F 0 "#PWR023" H 7600 5050 50  0001 C CNN
F 1 "GND" H 7605 5127 50  0000 C CNN
F 2 "" H 7600 5300 50  0001 C CNN
F 3 "" H 7600 5300 50  0001 C CNN
	1    7600 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 605FB89B
P 7600 4750
F 0 "#PWR022" H 7600 4500 50  0001 C CNN
F 1 "GND" H 7605 4577 50  0000 C CNN
F 2 "" H 7600 4750 50  0001 C CNN
F 3 "" H 7600 4750 50  0001 C CNN
	1    7600 4750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Top_Bottom J5
U 1 1 605FA5A7
P 7150 5200
F 0 "J5" H 7200 5417 50  0000 C CNN
F 1 "Conn_Sigout4" H 7200 5326 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 7150 5200 50  0001 C CNN
F 3 "~" H 7150 5200 50  0001 C CNN
	1    7150 5200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Top_Bottom J4
U 1 1 605CAB50
P 7150 4650
F 0 "J4" H 7200 4867 50  0000 C CNN
F 1 "Conn_Sigout3" H 7200 4776 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 7150 4650 50  0001 C CNN
F 3 "~" H 7150 4650 50  0001 C CNN
	1    7150 4650
	1    0    0    -1  
$EndComp
$Comp
L Analog_DAC:MCP4922 U1
U 1 1 603CBB0C
P 2300 5100
F 0 "U1" H 2600 5600 50  0000 C CNN
F 1 "MCP4922" H 2600 5500 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm_Socket_LongPads" H 3100 4800 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22250A.pdf" H 3100 4800 50  0001 C CNN
	1    2300 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60647E45
P 2300 5700
F 0 "#PWR0101" H 2300 5450 50  0001 C CNN
F 1 "GND" H 2305 5527 50  0000 C CNN
F 2 "" H 2300 5700 50  0001 C CNN
F 3 "" H 2300 5700 50  0001 C CNN
	1    2300 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 5700 2300 5600
NoConn ~ 1100 3550
Wire Wire Line
	5200 3000 5200 2850
NoConn ~ 1100 2350
NoConn ~ 1100 2450
NoConn ~ -6600 2850
$EndSCHEMATC