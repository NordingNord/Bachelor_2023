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
L Sensor_Pressure:40PC015G U1
U 1 1 63E50669
P 2800 1450
F 0 "U1" V 2233 1450 50  0000 C CNN
F 1 "Gravity" V 2324 1450 50  0000 C CNN
F 2 "" H 2900 1450 50  0001 C CNN
F 3 "http://www.honeywellscportal.com//index.php?ci_id=138832" H 2900 1450 50  0001 C CNN
	1    2800 1450
	0    1    1    0   
$EndComp
$Comp
L Sensor_Pressure:40PC015G U2
U 1 1 63E50D72
P 3750 1450
F 0 "U2" V 3183 1450 50  0000 C CNN
F 1 "Gravity" V 3274 1450 50  0000 C CNN
F 2 "" H 3850 1450 50  0001 C CNN
F 3 "http://www.honeywellscportal.com//index.php?ci_id=138832" H 3850 1450 50  0001 C CNN
	1    3750 1450
	0    1    1    0   
$EndComp
$Comp
L Sensor_Pressure:40PC015G U3
U 1 1 63E51588
P 4700 1450
F 0 "U3" V 4133 1450 50  0000 C CNN
F 1 "Gravity" V 4224 1450 50  0000 C CNN
F 2 "" H 4800 1450 50  0001 C CNN
F 3 "http://www.honeywellscportal.com//index.php?ci_id=138832" H 4800 1450 50  0001 C CNN
	1    4700 1450
	0    1    1    0   
$EndComp
$Comp
L Sensor_Pressure:40PC015G U4
U 1 1 63E51E06
P 5650 1450
F 0 "U4" V 5083 1450 50  0000 C CNN
F 1 "Gravity" V 5174 1450 50  0000 C CNN
F 2 "" H 5750 1450 50  0001 C CNN
F 3 "http://www.honeywellscportal.com//index.php?ci_id=138832" H 5750 1450 50  0001 C CNN
	1    5650 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 1450 2350 1450
Wire Wire Line
	2350 1450 2350 2250
Wire Wire Line
	3450 1450 3300 1450
Wire Wire Line
	3300 1450 3300 2250
Wire Wire Line
	4400 1450 4300 1450
Wire Wire Line
	4300 1450 4300 2250
Wire Wire Line
	5350 1450 5250 1450
Wire Wire Line
	5250 1450 5250 2250
$Comp
L power:GND #PWR?
U 1 1 63E56375
P 2350 2250
F 0 "#PWR?" H 2350 2000 50  0001 C CNN
F 1 "GND" H 2355 2077 50  0000 C CNN
F 2 "" H 2350 2250 50  0001 C CNN
F 3 "" H 2350 2250 50  0001 C CNN
	1    2350 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 63E56C01
P 3300 2250
F 0 "#PWR?" H 3300 2000 50  0001 C CNN
F 1 "GND" H 3305 2077 50  0000 C CNN
F 2 "" H 3300 2250 50  0001 C CNN
F 3 "" H 3300 2250 50  0001 C CNN
	1    3300 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 63E56FC0
P 4300 2250
F 0 "#PWR?" H 4300 2000 50  0001 C CNN
F 1 "GND" H 4305 2077 50  0000 C CNN
F 2 "" H 4300 2250 50  0001 C CNN
F 3 "" H 4300 2250 50  0001 C CNN
	1    4300 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 63E573B8
P 5250 2250
F 0 "#PWR?" H 5250 2000 50  0001 C CNN
F 1 "GND" H 5255 2077 50  0000 C CNN
F 2 "" H 5250 2250 50  0001 C CNN
F 3 "" H 5250 2250 50  0001 C CNN
	1    5250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1450 3100 2300
Wire Wire Line
	4050 1450 4150 1450
Wire Wire Line
	4150 1450 4150 2300
Wire Wire Line
	5000 1450 5100 1450
Wire Wire Line
	5100 1450 5100 2300
Wire Wire Line
	5950 1450 6100 1450
Wire Wire Line
	6100 1450 6100 2300
$Comp
L power:+5V #PWR?
U 1 1 63E5833B
P 3100 2300
F 0 "#PWR?" H 3100 2150 50  0001 C CNN
F 1 "+5V" H 3115 2473 50  0000 C CNN
F 2 "" H 3100 2300 50  0001 C CNN
F 3 "" H 3100 2300 50  0001 C CNN
	1    3100 2300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 63E58E62
P 4150 2300
F 0 "#PWR?" H 4150 2150 50  0001 C CNN
F 1 "+5V" H 4165 2473 50  0000 C CNN
F 2 "" H 4150 2300 50  0001 C CNN
F 3 "" H 4150 2300 50  0001 C CNN
	1    4150 2300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 63E5974D
P 5100 2300
F 0 "#PWR?" H 5100 2150 50  0001 C CNN
F 1 "+5V" H 5115 2473 50  0000 C CNN
F 2 "" H 5100 2300 50  0001 C CNN
F 3 "" H 5100 2300 50  0001 C CNN
	1    5100 2300
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 63E59CC7
P 6100 2300
F 0 "#PWR?" H 6100 2150 50  0001 C CNN
F 1 "+5V" H 6115 2473 50  0000 C CNN
F 2 "" H 6100 2300 50  0001 C CNN
F 3 "" H 6100 2300 50  0001 C CNN
	1    6100 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2800 1850 2800 2750
Wire Wire Line
	3750 1850 3750 2750
Wire Wire Line
	4700 1850 4700 2750
Wire Wire Line
	5650 1850 5650 2750
Text GLabel 2800 2750 3    50   Input ~ 0
A0
Text GLabel 3750 2750 3    50   Input ~ 0
A1
Text GLabel 4700 2750 3    50   Input ~ 0
A2
Text GLabel 5650 2750 3    50   Input ~ 0
A3
$EndSCHEMATC
