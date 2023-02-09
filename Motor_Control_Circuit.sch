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
L Driver_Motor:L293D U1
U 1 1 6394A4E8
P 5100 3650
F 0 "U1" H 5100 4831 50  0000 C CNN
F 1 "L293D" H 5100 4740 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 5350 2900 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 4800 4350 50  0001 C CNN
	1    5100 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3450 4150 3450
Text GLabel 4150 3450 0    50   Input ~ 0
GPIO12
Wire Wire Line
	4600 4050 4350 4050
Text GLabel 4150 4050 0    50   Input ~ 0
GPIO13
Wire Wire Line
	5000 2650 4850 2650
Wire Wire Line
	4850 2650 4850 2300
Wire Wire Line
	4850 2250 5100 2250
Wire Wire Line
	5100 2250 5100 2150
Wire Wire Line
	5200 2650 5350 2650
Wire Wire Line
	5350 2650 5350 2250
Wire Wire Line
	5350 2250 5100 2250
Connection ~ 5100 2250
Text GLabel 5100 2150 1    50   Input ~ 0
RASPY_5V
Wire Wire Line
	4600 3050 4350 3050
Text GLabel 4150 3050 0    50   Input ~ 0
GPIO21
Wire Wire Line
	4600 3250 4350 3250
Text GLabel 4150 3250 0    50   Input ~ 0
GPIO20
Wire Wire Line
	4600 3650 4350 3650
Text GLabel 4150 3650 0    50   Input ~ 0
GPIO6
Wire Wire Line
	4600 3850 4150 3850
Text GLabel 4150 3850 0    50   Input ~ 0
GPIO5
Wire Wire Line
	4900 4450 4900 4600
Wire Wire Line
	5000 4450 5000 4600
Connection ~ 4900 4600
Wire Wire Line
	5200 4450 5200 4600
Wire Wire Line
	4900 4600 5000 4600
Connection ~ 5000 4600
Wire Wire Line
	5300 4450 5300 4600
Wire Wire Line
	5000 4600 5200 4600
Connection ~ 5200 4600
Wire Wire Line
	5200 4600 5300 4600
$Comp
L power:GND #PWR?
U 1 1 639505C9
P 4300 4600
F 0 "#PWR?" H 4300 4350 50  0001 C CNN
F 1 "GND" V 4305 4472 50  0000 R CNN
F 2 "" H 4300 4600 50  0001 C CNN
F 3 "" H 4300 4600 50  0001 C CNN
	1    4300 4600
	0    1    1    0   
$EndComp
$Comp
L Motor:Motor_DC MR
U 1 1 63950BD2
P 6300 3100
F 0 "MR" H 6458 3096 50  0000 L CNN
F 1 "Motor_DC" H 6458 3005 50  0000 L CNN
F 2 "" H 6300 3010 50  0001 C CNN
F 3 "~" H 6300 3010 50  0001 C CNN
	1    6300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3050 5950 3050
Wire Wire Line
	5950 3050 5950 2900
Wire Wire Line
	5950 2900 6300 2900
Wire Wire Line
	5600 3250 5950 3250
Wire Wire Line
	5950 3250 5950 3400
Wire Wire Line
	5950 3400 6300 3400
$Comp
L Motor:Motor_DC ML
U 1 1 63952BFE
P 6300 3700
F 0 "ML" H 6458 3696 50  0000 L CNN
F 1 "Motor_DC" H 6458 3605 50  0000 L CNN
F 2 "" H 6300 3610 50  0001 C CNN
F 3 "~" H 6300 3610 50  0001 C CNN
	1    6300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3650 5950 3650
Wire Wire Line
	5950 3500 6300 3500
Wire Wire Line
	5600 3850 5950 3850
Wire Wire Line
	5950 3500 5950 3650
Wire Wire Line
	5950 3850 5950 4000
Wire Wire Line
	5950 4000 6300 4000
Wire Wire Line
	4300 4600 4900 4600
Wire Wire Line
	4350 3050 4350 2850
Connection ~ 4350 3050
Wire Wire Line
	4350 3050 4150 3050
$Comp
L Device:R R1000
U 1 1 639593C6
P 4200 2850
F 0 "R1000" V 3993 2850 50  0000 C CNN
F 1 "R" V 4084 2850 50  0000 C CNN
F 2 "" V 4130 2850 50  0001 C CNN
F 3 "~" H 4200 2850 50  0001 C CNN
	1    4200 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 2850 3850 2850
$Comp
L Device:LED DR1
U 1 1 6395A368
P 3700 2850
F 0 "DR1" H 3693 3067 50  0000 C CNN
F 1 "LED" H 3693 2976 50  0000 C CNN
F 2 "" H 3700 2850 50  0001 C CNN
F 3 "~" H 3700 2850 50  0001 C CNN
	1    3700 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2850 3400 2850
$Comp
L power:GND #PWR?
U 1 1 6395B45D
P 3400 2850
F 0 "#PWR?" H 3400 2600 50  0001 C CNN
F 1 "GND" V 3405 2722 50  0000 R CNN
F 2 "" H 3400 2850 50  0001 C CNN
F 3 "" H 3400 2850 50  0001 C CNN
	1    3400 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 3250 4350 3150
Connection ~ 4350 3250
Wire Wire Line
	4350 3250 4150 3250
$Comp
L power:GND #PWR?
U 1 1 6395DB1E
P 2900 3150
F 0 "#PWR?" H 2900 2900 50  0001 C CNN
F 1 "GND" V 2905 3022 50  0000 R CNN
F 2 "" H 2900 3150 50  0001 C CNN
F 3 "" H 2900 3150 50  0001 C CNN
	1    2900 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R1000
U 1 1 6395C825
P 3450 3150
F 0 "R1000" V 3243 3150 50  0000 C CNN
F 1 "R" V 3334 3150 50  0000 C CNN
F 2 "" V 3380 3150 50  0001 C CNN
F 3 "~" H 3450 3150 50  0001 C CNN
	1    3450 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 3150 3600 3150
$Comp
L Device:LED DR2
U 1 1 63960905
P 3150 3150
F 0 "DR2" H 3143 3367 50  0000 C CNN
F 1 "LED" H 3143 3276 50  0000 C CNN
F 2 "" H 3150 3150 50  0001 C CNN
F 3 "~" H 3150 3150 50  0001 C CNN
	1    3150 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 3150 2900 3150
Wire Wire Line
	4850 2300 4450 2300
Connection ~ 4850 2300
Wire Wire Line
	4850 2300 4850 2250
$Comp
L Device:R R1000
U 1 1 63962B44
P 4300 2300
F 0 "R1000" V 4093 2300 50  0000 C CNN
F 1 "R" V 4184 2300 50  0000 C CNN
F 2 "" V 4230 2300 50  0001 C CNN
F 3 "~" H 4300 2300 50  0001 C CNN
	1    4300 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 2300 3950 2300
$Comp
L Device:LED DP
U 1 1 63962B4B
P 3800 2300
F 0 "DP" H 3793 2517 50  0000 C CNN
F 1 "LED" H 3793 2426 50  0000 C CNN
F 2 "" H 3800 2300 50  0001 C CNN
F 3 "~" H 3800 2300 50  0001 C CNN
	1    3800 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 639638D6
P 3650 2300
F 0 "#PWR?" H 3650 2050 50  0001 C CNN
F 1 "GND" V 3655 2172 50  0000 R CNN
F 2 "" H 3650 2300 50  0001 C CNN
F 3 "" H 3650 2300 50  0001 C CNN
	1    3650 2300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 63964ECF
P 2950 3550
F 0 "#PWR?" H 2950 3300 50  0001 C CNN
F 1 "GND" V 2955 3422 50  0000 R CNN
F 2 "" H 2950 3550 50  0001 C CNN
F 3 "" H 2950 3550 50  0001 C CNN
	1    2950 3550
	0    1    1    0   
$EndComp
$Comp
L Device:R R1000
U 1 1 63964ED5
P 3500 3550
F 0 "R1000" V 3293 3550 50  0000 C CNN
F 1 "R" V 3384 3550 50  0000 C CNN
F 2 "" V 3430 3550 50  0001 C CNN
F 3 "~" H 3500 3550 50  0001 C CNN
	1    3500 3550
	0    1    1    0   
$EndComp
$Comp
L Device:LED DL1
U 1 1 63964EDB
P 3200 3550
F 0 "DL1" H 3193 3767 50  0000 C CNN
F 1 "LED" H 3193 3676 50  0000 C CNN
F 2 "" H 3200 3550 50  0001 C CNN
F 3 "~" H 3200 3550 50  0001 C CNN
	1    3200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3550 2950 3550
$Comp
L power:GND #PWR?
U 1 1 63966688
P 2950 3950
F 0 "#PWR?" H 2950 3700 50  0001 C CNN
F 1 "GND" V 2955 3822 50  0000 R CNN
F 2 "" H 2950 3950 50  0001 C CNN
F 3 "" H 2950 3950 50  0001 C CNN
	1    2950 3950
	0    1    1    0   
$EndComp
$Comp
L Device:R R1000
U 1 1 6396668E
P 3500 3950
F 0 "R1000" V 3293 3950 50  0000 C CNN
F 1 "R" V 3384 3950 50  0000 C CNN
F 2 "" V 3430 3950 50  0001 C CNN
F 3 "~" H 3500 3950 50  0001 C CNN
	1    3500 3950
	0    1    1    0   
$EndComp
$Comp
L Device:LED DL2
U 1 1 63966694
P 3200 3950
F 0 "DL2" H 3193 4167 50  0000 C CNN
F 1 "LED" H 3193 4076 50  0000 C CNN
F 2 "" H 3200 3950 50  0001 C CNN
F 3 "~" H 3200 3950 50  0001 C CNN
	1    3200 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3950 2950 3950
Wire Wire Line
	3650 3550 4350 3550
Wire Wire Line
	4350 3550 4350 3650
Connection ~ 4350 3650
Wire Wire Line
	4350 3650 4150 3650
Wire Wire Line
	3650 3950 4350 3950
Wire Wire Line
	4350 3950 4350 4050
Connection ~ 4350 4050
Wire Wire Line
	4350 4050 4150 4050
$EndSCHEMATC
