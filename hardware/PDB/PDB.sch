EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Ferdinand20 - Power Distribution Board"
Date "2020-07-06"
Rev "2"
Comp ""
Comment1 "https://creativecommons.org/licenses/by-sa/4.0/"
Comment2 "Released under the Creative Commons Attribution Share-Alike 4.0 License"
Comment3 "and remote deadman switch functionality."
Comment4 "Contains LCD (battery voltage, robot status, etc), fuses, "
$EndDescr
$Comp
L MyLibrary:N5150M5CD U3
U 1 1 5EF5A62B
P 5800 4900
F 0 "U3" H 5300 5500 50  0000 L CNN
F 1 "N5150M5CD" H 5300 5400 50  0000 L CNN
F 2 "adam_custom:adam_custom-N5150M4CD_TOP" H 5800 4900 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Dynastream%20PDFs/N5%20ANT%20SoC%20Module%20Series.pdf" H 5800 4900 50  0001 C CNN
	1    5800 4900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5EF5D99F
P 1000 2700
F 0 "J1" H 918 2375 50  0000 C CNN
F 1 "7.4V LiPo Battery" H 918 2466 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 1000 2700 50  0001 C CNN
F 3 "~" H 1000 2700 50  0001 C CNN
	1    1000 2700
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5EF5E778
P 10400 2900
F 0 "J3" H 10480 2892 50  0000 L CNN
F 1 "Motor Power" H 10480 2801 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 10400 2900 50  0001 C CNN
F 3 "~" H 10400 2900 50  0001 C CNN
	1    10400 2900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5EF5FB8D
P 10400 4650
F 0 "J5" H 10480 4692 50  0000 L CNN
F 1 "Power Switch" H 10480 4601 50  0000 L CNN
F 2 "Connector_Molex:Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical" H 10400 4650 50  0001 C CNN
F 3 "~" H 10400 4650 50  0001 C CNN
	1    10400 4650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5EF61B37
P 10400 5650
F 0 "J6" H 10480 5692 50  0000 L CNN
F 1 "Manual Switch" H 10480 5601 50  0000 L CNN
F 2 "Connector_Molex:Molex_SPOX_5267-03A_1x03_P2.50mm_Vertical" H 10400 5650 50  0001 C CNN
F 3 "~" H 10400 5650 50  0001 C CNN
	1    10400 5650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5EF626FF
P 10400 3650
F 0 "J4" H 10480 3642 50  0000 L CNN
F 1 "UART/Power" H 10480 3551 50  0000 L CNN
F 2 "Connector_Molex:Molex_SPOX_5267-04A_1x04_P2.50mm_Vertical" H 10400 3650 50  0001 C CNN
F 3 "~" H 10400 3650 50  0001 C CNN
	1    10400 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse_Small F1
U 1 1 5EF63D78
P 1900 2600
F 0 "F1" H 1900 2785 50  0000 C CNN
F 1 "15A Fuse" H 1900 2694 50  0000 C CNN
F 2 "MyLibrary:SMD_Keystone_Fuse" H 1900 2600 50  0001 C CNN
F 3 "~" H 1900 2600 50  0001 C CNN
	1    1900 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse_Small F2
U 1 1 5EF64BD3
P 2300 2350
F 0 "F2" H 2300 2535 50  0000 C CNN
F 1 "10A Fuse" H 2300 2444 50  0000 C CNN
F 2 "MyLibrary:SMD_Keystone_Fuse" H 2300 2350 50  0001 C CNN
F 3 "~" H 2300 2350 50  0001 C CNN
	1    2300 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse_Small F3
U 1 1 5EF65406
P 2300 3050
F 0 "F3" H 2300 3235 50  0000 C CNN
F 1 "2A Fuse" H 2300 3144 50  0000 C CNN
F 2 "MyLibrary:SMD_Keystone_Fuse" H 2300 3050 50  0001 C CNN
F 3 "~" H 2300 3050 50  0001 C CNN
	1    2300 3050
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MIC5205-3.3YM5 U1
U 1 1 5EF65956
P 3350 3600
F 0 "U1" H 3350 3942 50  0000 C CNN
F 1 "MIC5205-3.3YM5" H 3350 3851 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 3350 3925 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005785A.pdf" H 3350 3600 50  0001 C CNN
	1    3350 3600
	1    0    0    -1  
$EndComp
$Comp
L dk_PMIC-Voltage-Regulators-Linear:L4931ABD50-TR U2
U 1 1 5EF6D6F6
P 3400 2350
F 0 "U2" H 2950 3050 60  0000 L CNN
F 1 "L4931ABD50-TR" H 2950 2900 60  0000 L CNN
F 2 "digikey-footprints:SOIC-8_W3.9mm" H 3600 2550 60  0001 L CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/ff/95/2f/6a/75/70/42/49/CD00000971.pdf/files/CD00000971.pdf/jcr:content/translations/en.CD00000971.pdf" H 3600 2650 60  0001 L CNN
F 4 "497-1156-1-ND" H 3600 2750 60  0001 L CNN "Digi-Key_PN"
F 5 "L4931ABD50-TR" H 3600 2850 60  0001 L CNN "MPN"
F 6 "Integrated Circuits (ICs)" H 3600 2950 60  0001 L CNN "Category"
F 7 "PMIC - Voltage Regulators - Linear" H 3600 3050 60  0001 L CNN "Family"
F 8 "http://www.st.com/content/ccc/resource/technical/document/datasheet/ff/95/2f/6a/75/70/42/49/CD00000971.pdf/files/CD00000971.pdf/jcr:content/translations/en.CD00000971.pdf" H 3600 3150 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/stmicroelectronics/L4931ABD50-TR/497-1156-1-ND/586156" H 3600 3250 60  0001 L CNN "DK_Detail_Page"
F 10 "IC REG LINEAR 5V 250MA 8SO" H 3600 3350 60  0001 L CNN "Description"
F 11 "STMicroelectronics" H 3600 3450 60  0001 L CNN "Manufacturer"
F 12 "Active" H 3600 3550 60  0001 L CNN "Status"
	1    3400 2350
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Electromechanical:RELAY-SPDT-JQX-15F K1
U 1 1 5EF6F797
P 7600 2650
F 0 "K1" H 7550 3104 45  0000 C CNN
F 1 "RELAY-SPDT-JQX-15F" H 7550 3020 45  0000 C CNN
F 2 "SparkFun-Electromechanical:SparkFun-Electromechanical-RELAY-T90" H 7630 2800 20  0001 C CNN
F 3 "" H 7600 2650 50  0001 C CNN
	1    7600 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 5EF78998
P 6450 3150
F 0 "R1" V 6245 3150 50  0000 C CNN
F 1 "680 ohm" V 6336 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6490 3140 50  0001 C CNN
F 3 "~" H 6450 3150 50  0001 C CNN
	1    6450 3150
	0    1    1    0   
$EndComp
$Comp
L dk_Transistors-Bipolar-BJT-Single:MMBT2222ALT3G Q1
U 1 1 5EF79B71
P 6850 3150
F 0 "Q1" H 7038 3203 60  0000 L CNN
F 1 "MMBT2222ALT3G" H 7038 3097 60  0000 L CNN
F 2 "digikey-footprints:SOT-23-3" H 7050 3350 60  0001 L CNN
F 3 "http://www.onsemi.com/pub/Collateral/MMBT2222LT1-D.PDF" H 7050 3450 60  0001 L CNN
F 4 "MMBT2222ALT3GOSCT-ND" H 7050 3550 60  0001 L CNN "Digi-Key_PN"
F 5 "MMBT2222ALT3G" H 7050 3650 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7050 3750 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 7050 3850 60  0001 L CNN "Family"
F 8 "http://www.onsemi.com/pub/Collateral/MMBT2222LT1-D.PDF" H 7050 3950 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/on-semiconductor/MMBT2222ALT3G/MMBT2222ALT3GOSCT-ND/1967145" H 7050 4050 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 40V 0.6A SOT-23" H 7050 4150 60  0001 L CNN "Description"
F 11 "ON Semiconductor" H 7050 4250 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7050 4350 60  0001 L CNN "Status"
	1    6850 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D2
U 1 1 5EF7C82B
P 6950 2650
F 0 "D2" V 7000 2850 50  0000 R CNN
F 1 "SMAJ4739A-TP" V 6900 3300 50  0000 R CNN
F 2 "Diode_SMD:D_SMA" H 6950 2650 50  0001 C CNN
F 3 "~" H 6950 2650 50  0001 C CNN
	1    6950 2650
	0    -1   -1   0   
$EndComp
$Comp
L dk_Diodes-Rectifiers-Single:1N4148W-TP D1
U 1 1 5EF7D65A
P 6950 2150
F 0 "D1" V 7050 2300 60  0000 R CNN
F 1 "1N4148W-TP" V 6950 2800 60  0000 R CNN
F 2 "digikey-footprints:SOD-123" H 7150 2350 60  0001 L CNN
F 3 "https://www.mccsemi.com/pdf/Products/1N4148W(SOD123).pdf" H 7150 2450 60  0001 L CNN
F 4 "1N4148WTPMSCT-ND" H 7150 2550 60  0001 L CNN "Digi-Key_PN"
F 5 "1N4148W-TP" H 7150 2650 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 7150 2750 60  0001 L CNN "Category"
F 7 "Diodes - Rectifiers - Single" H 7150 2850 60  0001 L CNN "Family"
F 8 "https://www.mccsemi.com/pdf/Products/1N4148W(SOD123).pdf" H 7150 2950 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/micro-commercial-co/1N4148W-TP/1N4148WTPMSCT-ND/717311" H 7150 3050 60  0001 L CNN "DK_Detail_Page"
F 10 "DIODE GEN PURP 100V 150MA SOD123" H 7150 3150 60  0001 L CNN "Description"
F 11 "Micro Commercial Co" H 7150 3250 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7150 3350 60  0001 L CNN "Status"
	1    6950 2150
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x12 J2
U 1 1 5EF7E373
P 10400 1950
F 0 "J2" H 10480 1942 50  0000 L CNN
F 1 "LCD" H 10480 1851 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 10400 1950 50  0001 C CNN
F 3 "~" H 10400 1950 50  0001 C CNN
	1    10400 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5EFA6466
P 1550 2850
F 0 "#PWR0101" H 1550 2600 50  0001 C CNN
F 1 "GND" H 1555 2677 50  0000 C CNN
F 2 "" H 1550 2850 50  0001 C CNN
F 3 "" H 1550 2850 50  0001 C CNN
	1    1550 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0102
U 1 1 5EFA6F94
P 1550 1750
F 0 "#PWR0102" H 1550 1600 50  0001 C CNN
F 1 "+BATT" H 1565 1923 50  0000 C CNN
F 2 "" H 1550 1750 50  0001 C CNN
F 3 "" H 1550 1750 50  0001 C CNN
	1    1550 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5EFB899A
P 6950 3450
F 0 "#PWR0103" H 6950 3200 50  0001 C CNN
F 1 "GND" H 6955 3277 50  0000 C CNN
F 2 "" H 6950 3450 50  0001 C CNN
F 3 "" H 6950 3450 50  0001 C CNN
	1    6950 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 5EFC1739
P 6950 1850
F 0 "#PWR0104" H 6950 1700 50  0001 C CNN
F 1 "+5V" H 6965 2023 50  0000 C CNN
F 2 "" H 6950 1850 50  0001 C CNN
F 3 "" H 6950 1850 50  0001 C CNN
	1    6950 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 3350 6950 3450
Wire Wire Line
	6600 3150 6650 3150
Wire Wire Line
	6950 2950 6950 2850
Wire Wire Line
	7200 2850 6950 2850
Connection ~ 6950 2850
Wire Wire Line
	6950 2850 6950 2800
Wire Wire Line
	6950 2500 6950 2350
Wire Wire Line
	6950 1950 6950 1900
Wire Wire Line
	7200 2450 7100 2450
Wire Wire Line
	7100 2450 7100 1900
Wire Wire Line
	7100 1900 6950 1900
Connection ~ 6950 1900
Wire Wire Line
	6950 1900 6950 1850
$Comp
L Device:C C1
U 1 1 5EFC99E9
P 2800 2400
F 0 "C1" H 2650 2500 50  0000 L CNN
F 1 "0.1uF" H 2550 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2838 2250 50  0001 C CNN
F 3 "~" H 2800 2400 50  0001 C CNN
	1    2800 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5EFCA1FA
P 3800 2500
F 0 "C3" H 3915 2546 50  0000 L CNN
F 1 "1uF" H 3915 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3838 2350 50  0001 C CNN
F 3 "~" H 3800 2500 50  0001 C CNN
	1    3800 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EFCA805
P 4150 2500
F 0 "C4" H 4265 2546 50  0000 L CNN
F 1 "1uF" H 4265 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 2350 50  0001 C CNN
F 3 "~" H 4150 2500 50  0001 C CNN
	1    4150 2500
	1    0    0    -1  
$EndComp
NoConn ~ 3000 2350
$Comp
L power:GND #PWR0105
U 1 1 5EFCAFEA
P 3350 2850
F 0 "#PWR0105" H 3350 2600 50  0001 C CNN
F 1 "GND" H 3355 2677 50  0000 C CNN
F 2 "" H 3350 2850 50  0001 C CNN
F 3 "" H 3350 2850 50  0001 C CNN
	1    3350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2750 3200 2800
Wire Wire Line
	3200 2800 3300 2800
Wire Wire Line
	3500 2800 3500 2750
Wire Wire Line
	3350 2850 3350 2800
Connection ~ 3350 2800
Wire Wire Line
	3350 2800 3400 2800
Wire Wire Line
	3300 2750 3300 2800
Connection ~ 3300 2800
Wire Wire Line
	3300 2800 3350 2800
Wire Wire Line
	3400 2750 3400 2800
Connection ~ 3400 2800
Wire Wire Line
	3400 2800 3500 2800
Wire Wire Line
	1200 2600 1550 2600
Wire Wire Line
	1550 2600 1550 1900
Wire Wire Line
	1200 2700 1550 2700
Wire Wire Line
	1550 2700 1550 2850
Wire Wire Line
	1800 2600 1550 2600
Connection ~ 1550 2600
Wire Wire Line
	2000 2600 2100 2600
Wire Wire Line
	2100 2600 2100 2350
Wire Wire Line
	2100 2350 2200 2350
Connection ~ 2100 2600
Wire Wire Line
	2400 2350 2500 2350
Wire Wire Line
	2500 2350 2500 1950
Wire Wire Line
	2500 1950 2800 1950
Wire Wire Line
	3300 1950 3300 2050
Wire Wire Line
	2800 2250 2800 1950
Connection ~ 2800 1950
Wire Wire Line
	2800 1950 3300 1950
Wire Wire Line
	2800 2550 2800 2800
Wire Wire Line
	2800 2800 3200 2800
Connection ~ 3200 2800
$Comp
L power:+5V #PWR0106
U 1 1 5EFD34A7
P 3800 1750
F 0 "#PWR0106" H 3800 1600 50  0001 C CNN
F 1 "+5V" H 3815 1923 50  0000 C CNN
F 2 "" H 3800 1750 50  0001 C CNN
F 3 "" H 3800 1750 50  0001 C CNN
	1    3800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1750 3800 2250
Wire Wire Line
	3800 2250 3700 2250
Wire Wire Line
	3800 2350 3800 2250
Connection ~ 3800 2250
Wire Wire Line
	3800 2650 3800 2800
Wire Wire Line
	3800 2800 3500 2800
Connection ~ 3500 2800
Wire Wire Line
	4150 2350 4150 2250
Wire Wire Line
	4150 2250 3800 2250
Wire Wire Line
	4150 2650 4150 2800
Wire Wire Line
	4150 2800 3800 2800
Connection ~ 3800 2800
Wire Wire Line
	7900 2850 8000 2850
Text Notes 7850 2850 0    50   ~ 0
NO
Text Notes 7850 2400 0    50   ~ 0
NC
$Comp
L Device:C C2
U 1 1 5EFE8FFB
P 3750 3750
F 0 "C2" H 3865 3796 50  0000 L CNN
F 1 "470pF" H 3865 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3788 3600 50  0001 C CNN
F 3 "~" H 3750 3750 50  0001 C CNN
	1    3750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EFE9D9D
P 4150 3650
F 0 "C5" H 4265 3696 50  0000 L CNN
F 1 "1uF" H 4265 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 3500 50  0001 C CNN
F 3 "~" H 4150 3650 50  0001 C CNN
	1    4150 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5EFEA9E1
P 4550 3650
F 0 "C6" H 4665 3696 50  0000 L CNN
F 1 "1uF" H 4665 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4588 3500 50  0001 C CNN
F 3 "~" H 4550 3650 50  0001 C CNN
	1    4550 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3600 2950 3600
Wire Wire Line
	2950 3600 2950 3500
Connection ~ 2950 3500
Wire Wire Line
	2950 3500 3050 3500
$Comp
L power:GND #PWR0109
U 1 1 5EFEF026
P 3550 4000
F 0 "#PWR0109" H 3550 3750 50  0001 C CNN
F 1 "GND" H 3555 3827 50  0000 C CNN
F 2 "" H 3550 4000 50  0001 C CNN
F 3 "" H 3550 4000 50  0001 C CNN
	1    3550 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3900 3350 4000
Wire Wire Line
	3350 4000 3550 4000
Wire Wire Line
	3750 3900 3750 4000
Wire Wire Line
	3750 4000 3550 4000
Connection ~ 3550 4000
Wire Wire Line
	3650 3600 3750 3600
Wire Wire Line
	3650 3500 4150 3500
Wire Wire Line
	4550 3500 4150 3500
Connection ~ 4150 3500
Wire Wire Line
	4150 3800 4350 3800
Wire Wire Line
	3750 4000 4350 4000
Wire Wire Line
	4350 4000 4350 3800
Connection ~ 3750 4000
Connection ~ 4350 3800
Wire Wire Line
	4350 3800 4550 3800
$Comp
L power:+3.3V #PWR0110
U 1 1 5EFF9DDC
P 4550 1750
F 0 "#PWR0110" H 4550 1600 50  0001 C CNN
F 1 "+3.3V" H 4565 1923 50  0000 C CNN
F 2 "" H 4550 1750 50  0001 C CNN
F 3 "" H 4550 1750 50  0001 C CNN
	1    4550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1750 4550 3500
Connection ~ 4550 3500
$Comp
L power:+3.3V #PWR0111
U 1 1 5EFFE85B
P 10200 1300
F 0 "#PWR0111" H 10200 1150 50  0001 C CNN
F 1 "+3.3V" H 10215 1473 50  0000 C CNN
F 2 "" H 10200 1300 50  0001 C CNN
F 3 "" H 10200 1300 50  0001 C CNN
	1    10200 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 1300 10200 1450
Text GLabel 10150 1750 0    50   Input ~ 0
LCD_SCK
Text GLabel 10150 1850 0    50   Output ~ 0
LCD_MISO
Text GLabel 10150 1950 0    50   Input ~ 0
LCD_MOSI
Text GLabel 10150 2050 0    50   Input ~ 0
LCD_TFTCS
Text GLabel 10150 2150 0    50   Input ~ 0
LCD_TFTRST
Text GLabel 10150 2250 0    50   Input ~ 0
LCD_TFTDC
Text GLabel 10150 2450 0    50   Input ~ 0
LCD_LITE
Text GLabel 10150 2550 0    50   Output ~ 0
LCD_TE
$Comp
L power:GND #PWR0112
U 1 1 5F00CCB5
P 9600 1750
F 0 "#PWR0112" H 9600 1500 50  0001 C CNN
F 1 "GND" H 9605 1577 50  0000 C CNN
F 2 "" H 9600 1750 50  0001 C CNN
F 3 "" H 9600 1750 50  0001 C CNN
	1    9600 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1650 10200 1650
Wire Wire Line
	10150 1750 10200 1750
Wire Wire Line
	10150 1850 10200 1850
Wire Wire Line
	10150 1950 10200 1950
Wire Wire Line
	10150 2050 10200 2050
Wire Wire Line
	10150 2150 10200 2150
Wire Wire Line
	10150 2250 10200 2250
Wire Wire Line
	10150 2450 10200 2450
Wire Wire Line
	10150 2550 10200 2550
Text GLabel 6250 3150 0    50   Input ~ 0
MOTOR_ON
Wire Wire Line
	6250 3150 6300 3150
$Comp
L power:+3.3V #PWR0113
U 1 1 5F023176
P 5800 4250
F 0 "#PWR0113" H 5800 4100 50  0001 C CNN
F 1 "+3.3V" H 5815 4423 50  0000 C CNN
F 2 "" H 5800 4250 50  0001 C CNN
F 3 "" H 5800 4250 50  0001 C CNN
	1    5800 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4250 5800 4450
$Comp
L power:GND #PWR0114
U 1 1 5F026120
P 5800 6100
F 0 "#PWR0114" H 5800 5850 50  0001 C CNN
F 1 "GND" H 5805 5927 50  0000 C CNN
F 2 "" H 5800 6100 50  0001 C CNN
F 3 "" H 5800 6100 50  0001 C CNN
	1    5800 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 6000 5800 6100
Text GLabel 6200 4650 2    50   Input ~ 0
LCD_TE
Text GLabel 6200 4750 2    50   Output ~ 0
LCD_TFTDC
Text GLabel 6200 4850 2    50   Input ~ 0
UART_RXD
Text GLabel 6200 4950 2    50   Output ~ 0
UART_TXD
Text GLabel 6200 5050 2    50   Output ~ 0
LCD_SCK
Text GLabel 6200 5150 2    50   Output ~ 0
LCD_TFTCS
Text GLabel 6200 5250 2    50   Input ~ 0
LCD_MISO
Text GLabel 6200 5350 2    50   Output ~ 0
LCD_MOSI
Wire Wire Line
	6150 4650 6200 4650
Wire Wire Line
	6150 4750 6200 4750
Wire Wire Line
	6150 4850 6200 4850
Wire Wire Line
	6150 4950 6200 4950
Wire Wire Line
	6150 5050 6200 5050
Wire Wire Line
	6150 5150 6200 5150
Wire Wire Line
	6150 5250 6200 5250
Wire Wire Line
	6150 5350 6200 5350
Text GLabel 8100 2650 2    50   Output ~ 0
MOTOR_PWR
Text GLabel 10150 2900 0    50   Input ~ 0
MOTOR_PWR
Wire Wire Line
	10150 2900 10200 2900
Wire Wire Line
	7900 2650 8100 2650
Wire Wire Line
	9600 1750 9600 1650
$Comp
L power:GND #PWR0115
U 1 1 5F05E6E0
P 10200 3100
F 0 "#PWR0115" H 10200 2850 50  0001 C CNN
F 1 "GND" H 10205 2927 50  0000 C CNN
F 2 "" H 10200 3100 50  0001 C CNN
F 3 "" H 10200 3100 50  0001 C CNN
	1    10200 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3000 10200 3100
Text GLabel 10050 4550 0    50   Input ~ 0
PWR_SW_IN
Text GLabel 10050 4650 0    50   Output ~ 0
PWR_SW_OUT
$Comp
L Device:R_US R2
U 1 1 5F066F87
P 10000 4750
F 0 "R2" V 10100 4650 50  0000 C CNN
F 1 "1k ohm" V 10100 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10040 4740 50  0001 C CNN
F 3 "~" H 10000 4750 50  0001 C CNN
	1    10000 4750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5F0683AB
P 9800 4850
F 0 "#PWR0116" H 9800 4600 50  0001 C CNN
F 1 "GND" H 9805 4677 50  0000 C CNN
F 2 "" H 9800 4850 50  0001 C CNN
F 3 "" H 9800 4850 50  0001 C CNN
	1    9800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 4550 10200 4550
Wire Wire Line
	10150 4750 10200 4750
Wire Wire Line
	9850 4750 9800 4750
Wire Wire Line
	9800 4750 9800 4850
Text GLabel 10150 3550 0    50   Input ~ 0
PWR_SW_OUT
$Comp
L power:GND #PWR0117
U 1 1 5F074A71
P 10150 3950
F 0 "#PWR0117" H 10150 3700 50  0001 C CNN
F 1 "GND" H 10155 3777 50  0000 C CNN
F 2 "" H 10150 3950 50  0001 C CNN
F 3 "" H 10150 3950 50  0001 C CNN
	1    10150 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 3950 10150 3850
Wire Wire Line
	10150 3850 10200 3850
Wire Wire Line
	10150 3550 10200 3550
Text GLabel 2450 3050 2    50   Output ~ 0
PWR_SW_IN
Wire Wire Line
	2400 3050 2450 3050
Text GLabel 2900 3500 0    50   Input ~ 0
PWR_SW_OUT
Wire Wire Line
	2900 3500 2950 3500
Wire Wire Line
	2100 2600 2100 3050
Wire Wire Line
	2100 3050 2200 3050
$Comp
L power:+3.3V #PWR0118
U 1 1 5F099B28
P 10150 5450
F 0 "#PWR0118" H 10150 5300 50  0001 C CNN
F 1 "+3.3V" H 10165 5623 50  0000 C CNN
F 2 "" H 10150 5450 50  0001 C CNN
F 3 "" H 10150 5450 50  0001 C CNN
	1    10150 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 5F09B9FB
P 10000 5750
F 0 "R3" V 10100 5650 50  0000 C CNN
F 1 "1k ohm" V 10100 5900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10040 5740 50  0001 C CNN
F 3 "~" H 10000 5750 50  0001 C CNN
	1    10000 5750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5F09C8CD
P 9800 5900
F 0 "#PWR0119" H 9800 5650 50  0001 C CNN
F 1 "GND" H 9805 5727 50  0000 C CNN
F 2 "" H 9800 5900 50  0001 C CNN
F 3 "" H 9800 5900 50  0001 C CNN
	1    9800 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 5900 9800 5750
Wire Wire Line
	9800 5750 9850 5750
Wire Wire Line
	10150 5750 10200 5750
Wire Wire Line
	10150 5450 10150 5550
Wire Wire Line
	10150 5550 10200 5550
Text GLabel 10150 5650 0    50   Output ~ 0
MANUAL_SW
Wire Wire Line
	10150 5650 10200 5650
Text GLabel 6200 5750 2    50   Output ~ 0
MOTOR_ON
Wire Wire Line
	6150 5750 6200 5750
Text GLabel 6200 5550 2    50   Input ~ 0
MANUAL_SW
Text GLabel 6200 5650 2    50   Output ~ 0
LCD_TFTRST
Wire Wire Line
	6150 5550 6200 5550
Wire Wire Line
	6150 5650 6200 5650
Text GLabel 10150 3650 0    50   Output ~ 0
UART_RXD
Text GLabel 10150 3750 0    50   Input ~ 0
UART_TXD
Wire Wire Line
	10150 3650 10200 3650
Wire Wire Line
	10150 3750 10200 3750
NoConn ~ 5100 4650
NoConn ~ 5100 4750
Text Notes 9650 2400 0    50   ~ 0
LCD_CARDCS
NoConn ~ 10200 2350
Text Notes 9800 1600 0    50   ~ 0
LCD_3V3
NoConn ~ 10200 1550
NoConn ~ 7900 2450
Text Notes 9800 1500 0    50   ~ 0
LCD_VIN
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F117C42
P 1200 1750
F 0 "#FLG0101" H 1200 1825 50  0001 C CNN
F 1 "PWR_FLAG" H 1200 1923 50  0001 C CNN
F 2 "" H 1200 1750 50  0001 C CNN
F 3 "~" H 1200 1750 50  0001 C CNN
	1    1200 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1750 1200 1900
Wire Wire Line
	1200 1900 1550 1900
Connection ~ 1550 1900
Wire Wire Line
	1550 1900 1550 1750
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F11C715
P 1850 2850
F 0 "#FLG0102" H 1850 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 3023 50  0001 C CNN
F 2 "" H 1850 2850 50  0001 C CNN
F 3 "~" H 1850 2850 50  0001 C CNN
	1    1850 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1850 2850 1850 2700
Wire Wire Line
	1850 2700 1550 2700
Connection ~ 1550 2700
Text GLabel 2800 1700 1    50   Output ~ 0
Vmotor
Wire Wire Line
	2800 1700 2800 1950
Text GLabel 8000 1750 1    50   Input ~ 0
Vmotor
Wire Wire Line
	8000 1750 8000 2850
Wire Wire Line
	10050 4650 10200 4650
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5F137AD0
P 2950 3400
F 0 "#FLG0103" H 2950 3475 50  0001 C CNN
F 1 "PWR_FLAG" H 2950 3573 50  0001 C CNN
F 2 "" H 2950 3400 50  0001 C CNN
F 3 "~" H 2950 3400 50  0001 C CNN
	1    2950 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 3400 2950 3500
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5F13C8DF
P 2500 1750
F 0 "#FLG0104" H 2500 1825 50  0001 C CNN
F 1 "PWR_FLAG" H 2500 1923 50  0001 C CNN
F 2 "" H 2500 1750 50  0001 C CNN
F 3 "~" H 2500 1750 50  0001 C CNN
	1    2500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1750 2500 1950
Connection ~ 2500 1950
Text Notes 9500 950  0    50   ~ 0
Connects to Adafruit LCD Breakout:\nhttps://www.adafruit.com/product/3787
Text GLabel 6200 5850 2    50   Output ~ 0
LCD_LITE
Wire Wire Line
	6150 5850 6200 5850
NoConn ~ 6150 5450
$EndSCHEMATC
