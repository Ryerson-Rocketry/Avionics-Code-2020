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
L SparkFun-GPS:AULT_GPS GPS1
U 1 1 6025C995
P 5740 5970
F 0 "GPS1" V 5720 6060 50  0000 L CNN
F 1 "AULT_GPS" V 6000 5900 50  0000 L CNN
F 2 "Adafruit Ultimate GPS:1X09_ROUND_70" H 5690 5970 50  0001 C CNN
F 3 "" H 5690 5970 50  0001 C CNN
	1    5740 5970
	0    1    1    0   
$EndComp
$Comp
L Sensor_Pressure:BMP280 sensor1
U 1 1 602576E2
P 2470 6250
F 0 "sensor1" V 2680 5840 50  0000 L CNN
F 1 "BMP280" V 2680 6270 50  0000 L CNN
F 2 "Connectors:1X07" H 2470 5550 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001.pdf" H 2470 6250 50  0001 C CNN
	1    2470 6250
	0    1    -1   0   
$EndComp
Wire Wire Line
	5390 5670 5390 5770
Wire Wire Line
	5280 5580 5290 5770
$Comp
L SparkFun-Sensors:ADXL377 sensor2
U 1 1 60259330
P 5260 6840
F 0 "sensor2" H 5740 7220 45  0000 C CNN
F 1 "ADXL377" H 5260 7316 45  0000 C CNN
F 2 "Connectors:1X06" H 5260 7290 20  0001 C CNN
F 3 "" H 5260 6840 60  0001 C CNN
F 4 "IC-12027" H 5260 7221 60  0000 C CNN "Field4"
	1    5260 6840
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4090 4730 3990 4730
Wire Wire Line
	800  4050 1690 4030
Wire Wire Line
	800  4050 800  7640
Wire Wire Line
	650  3960 1690 3930
Wire Wire Line
	6130 7610 6130 5660
Wire Wire Line
	5390 5670 6130 5660
Wire Wire Line
	800  7640 6130 7610
Wire Wire Line
	650  7810 6430 7780
Wire Wire Line
	6430 7780 6420 5580
Wire Wire Line
	5280 5580 6420 5580
Wire Wire Line
	650  3960 650  7810
Wire Wire Line
	2720 6650 4090 6650
Wire Wire Line
	4760 6540 4200 6540
Wire Wire Line
	4200 6540 4200 4630
Wire Wire Line
	3990 4630 4200 4630
Wire Wire Line
	4760 6640 4090 6640
Wire Wire Line
	4090 4730 4090 6640
Connection ~ 4090 6640
Wire Wire Line
	4090 6640 4090 6650
Wire Wire Line
	2520 6650 2520 7220
Wire Wire Line
	2520 7220 1360 7230
Wire Wire Line
	1360 7230 1360 4840
Wire Wire Line
	1360 4840 1690 4830
Wire Wire Line
	2220 6650 2220 6770
Wire Wire Line
	2220 6770 1690 6770
Wire Wire Line
	1690 6770 1690 5330
Wire Wire Line
	2420 6650 2420 7090
Wire Wire Line
	2420 7090 1530 7090
Wire Wire Line
	1530 7090 1530 5230
Wire Wire Line
	1530 5230 1690 5230
Wire Wire Line
	2560 7330 1160 7330
Wire Wire Line
	1160 7330 1160 4730
Wire Wire Line
	1160 4730 1690 4730
Wire Wire Line
	2560 6740 2560 7330
Wire Wire Line
	4760 6740 2560 6740
Wire Wire Line
	4760 6840 2600 6840
Wire Wire Line
	2600 6840 2600 7420
Wire Wire Line
	2600 7420 1080 7420
Wire Wire Line
	1080 7420 1080 4630
Wire Wire Line
	1080 4630 1690 4630
Wire Wire Line
	4760 6940 2660 6940
Wire Wire Line
	2660 6940 2660 7500
Wire Wire Line
	2660 7500 1000 7500
Wire Wire Line
	1000 7500 1000 4530
Wire Wire Line
	1000 4530 1690 4530
$Comp
L Device:CP C2
U 1 1 6028C494
P 6830 4680
F 0 "C2" V 6960 4650 50  0000 L CNN
F 1 "10uF" V 6920 4360 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_3x5.3" H 6868 4530 50  0001 C CNN
F 3 "~" H 6830 4680 50  0001 C CNN
	1    6830 4680
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6680 4680 6680 4500
Wire Wire Line
	6680 4500 6710 4500
Wire Wire Line
	6980 4680 7010 4200
$Comp
L Device:C C1
U 1 1 60298910
P 6830 4840
F 0 "C1" V 6716 4794 50  0000 R CNN
F 1 "0.1uF" V 6630 4850 50  0000 R CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 6868 4690 50  0001 C CNN
F 3 "~" H 6830 4840 50  0001 C CNN
	1    6830 4840
	0    1    -1   0   
$EndComp
Wire Wire Line
	6680 4840 6680 4680
Connection ~ 6680 4680
Wire Wire Line
	6980 4840 6980 4680
Connection ~ 6980 4680
Connection ~ 6680 4840
Connection ~ 7010 4200
Connection ~ 6980 4840
$Comp
L teensy:Teensy3.6 microcontroller1
U 1 1 60257C60
P 2840 3180
F 0 "microcontroller1" H 2840 5617 60  0000 C CNN
F 1 "Teensy3.6" H 2840 5511 60  0000 C CNN
F 2 "teensy:Teensy35_36" H 2840 3230 60  0001 C CNN
F 3 "" H 2840 3230 60  0000 C CNN
	1    2840 3180
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell Battery1
U 1 1 6025A213
P 6900 3130
F 0 "Battery1" V 7090 3110 50  0000 L CNN
F 1 "4.5V" V 7018 3135 50  0000 L CNN
F 2 "Battery:BatteryHolder_Keystone_2479_3xAAA" V 6900 3190 50  0001 C CNN
F 3 "~" V 6900 3190 50  0001 C CNN
	1    6900 3130
	0    -1   -1   0   
$EndComp
Connection ~ 7000 3770
Wire Wire Line
	7010 4200 7000 3770
Wire Wire Line
	6710 3900 6700 3770
Connection ~ 6700 3770
Wire Wire Line
	6700 3130 6700 3410
$Comp
L Device:CP C3
U 1 1 6023258A
P 6850 3770
F 0 "C3" V 6930 3460 50  0000 L CNN
F 1 "10uF" V 6840 3390 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_3x5.3" H 6888 3620 50  0001 C CNN
F 3 "~" H 6850 3770 50  0001 C CNN
	1    6850 3770
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6700 3410 6700 3770
Wire Wire Line
	7000 3410 7000 3770
Connection ~ 7000 3410
Connection ~ 6700 3410
$Comp
L Device:Battery_Cell Battery2
U 1 1 60317B3E
P 6900 3410
F 0 "Battery2" V 6720 3410 50  0000 L CNN
F 1 "4.5V" V 6810 3390 50  0000 L CNN
F 2 "Battery:BatteryHolder_Keystone_2479_3xAAA" V 6900 3470 50  0001 C CNN
F 3 "~" V 6900 3470 50  0001 C CNN
	1    6900 3410
	0    -1   -1   0   
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 linear_volt_reg1
U 1 1 6024F7BB
P 6710 4200
F 0 "linear_volt_reg1" H 6710 4442 50  0000 C CNN
F 1 "LM7805_TO220" H 6710 4351 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6710 4425 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 6710 4150 50  0001 C CNN
	1    6710 4200
	0    -1   1    0   
$EndComp
Wire Wire Line
	5190 5420 5190 5770
Wire Wire Line
	4390 5030 4390 4530
Wire Wire Line
	4390 4530 3990 4530
Wire Wire Line
	5090 5770 5090 5610
Wire Wire Line
	5090 5610 4470 5610
Wire Wire Line
	4470 5610 4470 5030
Connection ~ 4470 5030
Wire Wire Line
	4470 5030 4390 5030
Wire Wire Line
	5190 5420 6980 5420
Wire Wire Line
	6980 4840 6980 5420
Wire Wire Line
	6680 4840 6680 5030
Wire Wire Line
	4470 5030 5210 5030
Connection ~ 5210 5030
Wire Wire Line
	5210 5030 6680 5030
Wire Wire Line
	7000 2870 6340 2870
Wire Wire Line
	6340 2870 6340 4590
Wire Wire Line
	6340 4590 5310 4590
Wire Wire Line
	7000 2870 7000 3130
Connection ~ 7000 3130
Wire Wire Line
	7000 3130 7000 3410
Wire Wire Line
	4610 4060 4290 4060
Wire Wire Line
	4290 4060 4290 3530
Wire Wire Line
	4290 3530 3990 3530
Wire Wire Line
	1450 1130 1690 1130
Wire Wire Line
	1270 240  1270 1230
Wire Wire Line
	5210 3970 5210 5030
Wire Wire Line
	5310 4590 5310 3970
Wire Wire Line
	4610 3970 4610 4060
Wire Wire Line
	5010 240  1270 240 
$Comp
L RF_Module:RFD900+ RF1
U 1 1 601DEA0B
P 5310 3070
F 0 "RF1" H 5770 3410 50  0000 R CNN
F 1 "RFD900+" H 5820 3320 50  0000 R CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x08_P2.54mm_Horizontal" H 5310 3070 50  0001 C CNN
F 3 "" H 5310 3070 50  0001 C CNN
	1    5310 3070
	0    1    1    0   
$EndComp
Wire Wire Line
	5010 2820 5010 240 
Wire Wire Line
	1450 480  1450 1130
Wire Wire Line
	4910 2820 4910 480 
Wire Wire Line
	4910 480  1450 480 
Wire Wire Line
	1270 1230 1690 1230
$EndSCHEMATC
