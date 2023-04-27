# -------------------- A Collected File with all necessary code ----------------------

# ---------------------------- Imports------------------------------------------------
import smbus
import time
import math
import RPi.GPIO as GPIO

# ---------------------------- Create I2C bus ----------------------------------------
Bus = smbus.SMBus(1)

# ---------------------------- Constants ---------------------------------------------

# Address definitions
Address_48 = 0x48
Address_49 = 0x49

# Conversion register
Reg_Pointer_Convert = 0x00
# Configuration register
Reg_Pointer_Config = 0x01
# Lower threshold register
Reg_Pointer_Low_Thresh = 0x02
# High threshold regiser
Reg_Pointer_High_Thresh = 0x03

# Configuration registers:
# No effect
Reg_No_Effect = 0x00
# Single convertion register
Reg_Config_Single = 0x80
# Differential A0 and A1
Reg_Config_Diff_0_1 = 0x00
# Differential A0 and A3
Reg_Config_Diff_0_3 = 0x10
# Differential A1 and A3
Reg_Config_Diff_1_3 = 0x20
# Differential A2 and A3
Reg_Config_Diff_2_3 = 0x30
# Single A0
Reg_Config_Single_0 = 0x40
# Single A1
Reg_Config_Single_1 = 0x50
# Single A2
Reg_Config_Single_2 = 0x60
# Single A3
Reg_Config_Single_3 = 0x70

# Range +/- 6.144V range = gain 2/3
Reg_Config_Range_6_144V = 0x00
# Range +/- 4.096V range = gain 1
Reg_Config_Range_4_096V = 0x02
# Range +/- 2.048 range = gain 2
Reg_Config_Range_2_048V = 0x04
# Range +/- 1.024V range = gain 4
Reg_Config_Range_1_024V = 0x06
# Range +/- 0.512V range = gain 8
Reg_Config_Range_0_512V = 0x08
# Range +/- 0.256V range = gain 16
Reg_Config_Range_0_256V = 0x0A

# Continuous conversion mode
Reg_Config_Mode_Continuous = 0x00
# Power down single shot mode
Reg_Config_Mode_Single = 0x01

# 8 Samples per second
Reg_Config_8SPS = 0x00
# 16 samples per second
Reg_Config_16SPS = 0x20
# 32 samples per second
Reg_Config_32SPS = 0x40
# 64 samples per second
Reg_Config_64SPS = 0x60
# 128 samples per second
Reg_Config_128SPS = 0x80
# 250 samples per second
Reg_Config_250SPS = 0xA0
# 475 samples per second
Reg_Config_475SPS = 0xC0
# 860 samples per second
Reg_Config_860SPS = 0xE0

# Traditional comparator (hysteresis)
Reg_Config_Comp_Mode_Trad = 0x00
# Window comparator
Reg_Config_Comp_Mode_Window = 0x10

# Alert/ready pin low when active
Reg_Config_Active_Low = 0x00
# Alert/ready pin high when active
Reg_Config_Active_High = 0x08

# Non latch comparator
Reg_Config_Non_Latch = 0x00
# Latching comparator
Reg_Config_Latch = 0x04

# Assert Alert/ready after one conversion
Reg_Config_Alert_1 = 0x00
# Assert Alert/ready after two conversions
Reg_Config_Alert_2 = 0x01
# Assert Alert/ready after four conversions
Reg_Config_Alert_4 = 0x02
# Disable comparator and put Alert/ready in high state
Reg_Cofig_Alert_None = 0x03

# Some defined numbers
Max_16_Bit = 65535

# ---------------------------- Global Variables ----------------------------------------

# Variables related to ADC settup and reading
My_Gain = 0
My_Coefficient = 0
My_Address = 0
My_Channel = 0
Config_Reg = []

# Scaler in regards to pressure reading
Scaler = 250

# Defined sample sizes
Samples = 100

# Delay times
Delay_Short = 0.1
Delay_Long = 0.2

# Text padding
Padding_Long = 25
Padding_Short = 6

# Defined channels
Channel_S1 = 0
Channel_S2 = 1
Channel_S3 = 2

# Measurements in regards to sensor pipe
Radius = 0.0225 # meters
Dynamic_Viscosity = 8.90*pow(10,-4) # Pascal per second
Length = 0.165 # meters
Area = math.pi*pow(Radius,2)
Density_Water = 998 # kg/m^3
Gravity = 9.815

# Variables important to the sensors
Sensor_Offsets = [0.497375,0.4998125,0.4991875, 0.496, 0.496, 0.496]
MVA_Raw_S1 = []
MVA_Raw_S2 = []
MVA_Raw_S3 = []
Index = 0
MVA_Samples = 5

# Variables for motor control
Motor_1_En = 12 #26 #12
Motor_1_R = 21 #29 #21 
Motor_1_L = 20 #28 #20 

Motor_2_En = 13 #23 #13
Motor_2_R = 6 #22 #6 
Motor_2_L = 5 #21 #5 

# ---------------------------- ADC Setup Methods -------------------------------------

# Method that sets our gain
def Set_Gain(Gain):
    # Update my gain
    global My_Gain
    global My_Coefficient
    My_Gain = Gain
    # Use this to define coefficient needed
    if My_Gain == Reg_Config_Range_6_144V:
        My_Coefficient = 0.1875
    elif My_Gain == Reg_Config_Range_4_096V:
        My_Coefficient = 0.125
    elif My_Gain == Reg_Config_Range_2_048V:
        My_Coefficient = 0.0625
    elif My_Gain == Reg_Config_Range_1_024V:
        My_Coefficient = 0.03125 # LSB but in milivolts
    elif My_Gain == Reg_Config_Range_0_512V:
        My_Coefficient = 0.015625
    elif  My_Gain == Reg_Config_Range_0_256V:
        My_Coefficient = 0.0078125
    else:
        My_Coefficient = 0.125

# Method that sets address
def Set_Address(Address):
    global My_Address
    My_Address = Address
    
# Method that sets channel
def Set_Channel(Channel):
    global My_Channel
    My_Channel = Channel
    
    # Make sure channel is 3 or smaller
    while My_Channel > 3:
        My_Channel = 0
    
    return My_Channel  

# Method that sets up single read
def Set_Single():
    # Define config register based on what channel we use
    global Config_Reg
    global My_Address
    if My_Channel == 0:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_0 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_128SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 1:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_1 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_128SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 2:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_2 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_128SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 3:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_3 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_128SPS | Reg_Cofig_Alert_None]
    
    # Write to bus
    Bus.write_i2c_block_data(My_Address, Reg_Pointer_Config, Config_Reg)

# Method that reads values
def Read_Value():
    global My_Address
    global My_Coefficient
    Data = Bus.read_i2c_block_data(My_Address,Reg_Pointer_Convert,2)
    # Convert data into raw adc data
    Raw_Data = Data[0]*256+Data[1] # 256 is in order to shift 8 bits. This is the ADC value
    # If value above half of max, then turn into minus version
    if Raw_Data > Max_16_Bit/2:
        Raw_Data -= Max_16_Bit
    
    return Raw_Data

def Get_Voltage(Raw_Data):
    
    # Make float times coeffcient
    Voltage = int(float(Raw_Data*My_Coefficient)) #float(Raw_Data*My_Coefficient)
    
    # Convert to V from mV
    Voltage = Voltage*pow(10,-3)
    
    # Return
    return Voltage

# ---------------------------- Motor Methods ------------------------------------------

# Setup GPIO
def GPIO_Setup():
    # Set GPIO numbering scheme
    GPIO.setmode(GPIO.BCM)
    # Set Needed pins as output
    GPIO.setup(Motor_1_En, GPIO.OUT)
    GPIO.setup(Motor_1_R, GPIO.OUT)
    GPIO.setup(Motor_1_L, GPIO.OUT)
    GPIO.setup(Motor_2_En, GPIO.OUT)
    GPIO.setup(Motor_2_R, GPIO.OUT)
    GPIO.setup(Motor_2_L, GPIO.OUT)

# Method for moving the boat forwards
def Move_Forward():
    # Enable both motors
    GPIO.output(Motor_1_En, GPIO.HIGH)
    GPIO.output(Motor_2_En, GPIO.HIGH)
    
    # Enable one direction
    GPIO.output(Motor_1_R, GPIO.HIGH)
    GPIO.output(Motor_2_R, GPIO.HIGH)
    
    # Disable the other
    GPIO.output(Motor_1_L, GPIO.LOW)
    GPIO.output(Motor_2_L, GPIO.LOW)

# Method for stopping the boat
def Move_Stop():
    # Disable the enable pins
    GPIO.output(Motor_1_En, GPIO.LOW)
    GPIO.output(Motor_2_En, GPIO.LOW)
    
    # Disable direction
    GPIO.output(Motor_1_R, GPIO.LOW)
    GPIO.output(Motor_2_R, GPIO.LOW)
    GPIO.output(Motor_1_L, GPIO.LOW)
    GPIO.output(Motor_2_L, GPIO.LOW)

# Method for moving the boat backwards
def Move_Backwards():
    # Enable both motors
    GPIO.output(Motor_1_En, GPIO.HIGH)
    GPIO.output(Motor_2_En, GPIO.HIGH)
    
    # Enable one direction and disable the other
    GPIO.output(Motor_1_R, GPIO.LOW)
    GPIO.output(Motor_2_R, GPIO.LOW)

    GPIO.output(Motor_1_L, GPIO.HIGH)
    GPIO.output(Motor_2_L, GPIO.HIGH)

# Method for moving the boat right
def Move_Right():
    # Disable Left motor
    GPIO.output(Motor_1_En, GPIO.LOW)
    GPIO.output(Motor_2_En, GPIO.HIGH)
    
    # Enable right movement
    GPIO.output(Motor_2_R, GPIO.HIGH)
    GPIO.output(Motor_2_L, GPIO.LOW)

# Method for moving the boat left
def Move_Left():
    # Disable Right motor
    GPIO.output(Motor_1_En, GPIO.HIGH)
    GPIO.output(Motor_2_En, GPIO.LOW)
    
    # Enable right movement
    GPIO.output(Motor_1_R, GPIO.HIGH)
    GPIO.output(Motor_1_L, GPIO.LOW)
    
# ---------------------------- Math Methods ------------------------------------------

# Method that calculates velocity from two pressure readings
def Calc_Velocity(Pressure_1, Pressure_2):
    # Calculate flow rate
    Flow_Rate = (math.pi*pow(Radius,4)*(Pressure_1-Pressure_2))/(8*Dynamic_Viscosity*Length)
    
    # Calculate Velocity
    Velocity = Flow_Rate/Area
    
    return Velocity

# Method that calculates Height change from two pressure and velocity readings
def Calc_Height(Pressure_1,Pressure_2, Velocity_1, Velocity_2):
    # Use Bernoulli's principle to determine change in height
    # Calculate equation 1, using height 0
    Value_1 = Pressure_1+(1/2)*Density_Water*pow(Velocity_1,2)
    # Calculate part of equation 2 that does not include height
    Value_2 = Pressure_2+(1/2)*Density_Water*pow(Velocity_2,2)
    # minus the two
    Height_Difference = Value_1-Value_2
    # Devide with values timed onto height
    Height_Difference = Height_Difference/(Density_Water*Gravity)
    # Convert from meters to cm
    Height_Difference = Height_Difference*100
    
    return Height_Difference

# ---------------------------- Data handling Methods ----------------------------------

# Method that reads voltage value for desired sensor
def Get_Reading(Sensor):
    
    Offset = 0
    
    # Set channel that matches the sensor
    if Sensor == "S1":
        Set_Channel(Channel_S1)
        Offset = Sensor_Offsets[0]
    elif Sensor == "S2":
        Set_Channel(Channel_S2)
        Offset = Sensor_Offsets[1]
    elif Sensor == "S3":
        Set_Channel(Channel_S3)
        Offset = Sensor_Offsets[2]
    else:
        print("Unknown sensor")
        return
    # Set mode to single
    Set_Single()
    
    # Sleep for a bit
    time.sleep(Delay_Short)
    
    # Get ADC Value
    Raw_Data = Read_Value()
    
    # Define voltage
    Voltage = Get_Voltage(Raw_Data)
    
    # Define Pressure
    Pressure = (Voltage-Offset)*Scaler
    
    # Collect Data into Dict
    Result = {"Raw":Raw_Data,"Voltage":Voltage,"Pressure":Pressure}
                  
    # Long sleep
    time.sleep(Delay_Long)
         
    # Return result
    return Result

def Moving_Average_Filter():
    global Index
    
    # Get readings
    S1_Readings = Get_Reading("S1")
    S2_Readings = Get_Reading("S2")
    S3_Readings = Get_Reading("S3")
    
    # Insert newest values
    if len(MVA_Raw_S1) < 10:
        MVA_Raw_S1.append(S1_Readings["Raw"])
        MVA_Raw_S2.append(S2_Readings["Raw"])
        MVA_Raw_S3.append(S3_Readings["Raw"])
    else:
        MVA_Raw_S1[Index] = S1_Readings["Raw"]
        MVA_Raw_S2[Index] = S2_Readings["Raw"]
        MVA_Raw_S3[Index] = S3_Readings["Raw"]
    
    # Calculate Average
    S1_Average = 0
    S2_Average = 0
    S3_Average = 0
    
    for i in range(len(MVA_Raw_S1)):
        S1_Average += MVA_Raw_S1[i]
        S2_Average += MVA_Raw_S2[i]
        S3_Average += MVA_Raw_S3[i]
        
    S1_Average = S1_Average/len(MVA_Raw_S1)
    S2_Average = S2_Average/len(MVA_Raw_S2)
    S3_Average = S3_Average/len(MVA_Raw_S3)
        
    # Calculate Voltage for average readings
    S1_V_Average = Get_Voltage(S1_Average)
    S2_V_Average = Get_Voltage(S2_Average)
    S3_V_Average = Get_Voltage(S3_Average)
    
    # Calculate Pressure for average readings
    S1_P_Average = (S1_V_Average-Sensor_Offsets[3])*Scaler
    S2_P_Average = (S2_V_Average-Sensor_Offsets[4])*Scaler
    S3_P_Average = (S3_V_Average-Sensor_Offsets[5])*Scaler
    
    # Collect into dict
    S1_Average_Readings = {"Raw":S1_Average,"Voltage":S1_V_Average,"Pressure":S1_P_Average}
    S2_Average_Readings = {"Raw":S2_Average,"Voltage":S2_V_Average,"Pressure":S2_P_Average}
    S3_Average_Readings = {"Raw":S3_Average,"Voltage":S3_V_Average,"Pressure":S3_P_Average}
    
    # Update index
    if Index == MVA_Samples-1:
        Index = 0
    else:
        Index += 1
    
    return S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings

def Print_Data(S1_Readings, S2_Readings, S3_Readings, Heights):
    
    # Write data to file
    Test_File = open("Final_Tests","a")
    
    # Write Raw Values
    Test_File.write("\nRaw Data: ")
    Test_File.write("\nS1 Raw Data: " + str(S1_Readings["Raw"]))
    Test_File.write("\nS2 Raw Data: " + str(S2_Readings["Raw"]))
    Test_File.write("\nS3 Raw Data: " + str(S3_Readings["Raw"]))
    
    # Write Voltage Values
    Test_File.write("\nVoltage Data: ")
    Test_File.write("\nS1 Voltage Data: " + str(S1_Readings["Voltage"]))
    Test_File.write("\nS2 Voltage Data: " + str(S2_Readings["Voltage"]))
    Test_File.write("\nS3 Voltage Data: " + str(S3_Readings["Voltage"]))
    
    # Write Pressure Values
    Test_File.write("\nPressure Data: ")
    Test_File.write("\nS1 Pressure Data: " + str(S1_Readings["Pressure"]))
    Test_File.write("\nS2 Pressure Data: " + str(S2_Readings["Pressure"]))
    Test_File.write("\nS3 Pressure Data: " + str(S3_Readings["Pressure"]))
    
    # Write calculated heights
    Test_File.write("\nCalculated Heights: ")
    Test_File.write(str(Heights))
    
    # Close file
    Test_File.close()
    
def Sample_Test():
    # Setup work dicts
    S1_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S2_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S3_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    Heights = []
    Velocity = 0
    Velocity_Old = 0
    Direction_Counter = 0
    Current_Direction = "Forward"
    
    # Get desired amount of data
    for i in range(Samples):
        # Get Data
        S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings = Moving_Average_Filter()
        # Update work dicts
        S1_Readings_T["Raw"].append(S1_Average_Readings["Raw"])
        S1_Readings_T["Voltage"].append(S1_Average_Readings["Voltage"])
        S1_Readings_T["Pressure"].append(S1_Average_Readings["Pressure"])
        
        S2_Readings_T["Raw"].append(S2_Average_Readings["Raw"])
        S2_Readings_T["Voltage"].append(S2_Average_Readings["Voltage"])
        S2_Readings_T["Pressure"].append(S2_Average_Readings["Pressure"])
        
        S3_Readings_T["Raw"].append(S3_Average_Readings["Raw"])
        S3_Readings_T["Voltage"].append(S3_Average_Readings["Voltage"])
        S3_Readings_T["Pressure"].append(S3_Average_Readings["Pressure"])
        
        # Calculate velocity
        Velocity_Old = Velocity
        Velocity = Calc_Velocity(S1_Average_Readings["Pressure"], S2_Average_Readings["Pressure"])
        
        # If more than one readings calculate height
        if len(S3_Readings_T["Raw"]) > 1:
            Heights.append(Calc_Height(S2_Readings_T["Pressure"][-1],S2_Readings_T["Pressure"][-2], Velocity, Velocity_Old))
        
        # print current sample
        print(i)
        Direction_Counter += 1
        if Direction_Counter == 50:
            Direction_Counter = 0
            if Current_Direction == "Forward":
                Move_Backwards()
                Current_Direction = "Backwards"
            elif Current_Direction == "Backwards":
                Move_Forward()
                Current_Direction = "Forward"
        elif abs(Heights[-1]) > 1:
            if Current_Direction == "Forward":
                Move_Backwards()
                Current_Direction = "Backwards"
            elif Current_Direction == "Backwards":
                Move_Forward()
                Current_Direction = "Forward"
            
    # print results to file
    Print_Data(S1_Readings_T, S2_Readings_T, S3_Readings_T, Heights)

# --------------------------------- Main ----------------------------------------------

# Set address
Set_Address(Address_48)
# Set gain and input voltage range
Set_Gain(Reg_Config_Range_1_024V)
# Setup GPIO
GPIO_Setup()

# Run test
Move_Forward()

Sample_Test()

Move_Stop()