#---- Imports ----
import smbus
import time
import math

#----- Definitions -----

# The bus
Bus = smbus.SMBus(1) # Why 1?

# Address definitions
Address_48 = 0x48
Address_49 = 0x49

# Address mapping:

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

# ------ Global variables ------------
My_Gain = 0
My_Coefficient = 0
My_Address = 0
My_Channel = 0
Config_Reg = []

Max_16_Bit = 65535
Reference_V = 1.1
Scaler = 250
Samples = 2000
Padding_Long = 25
Padding_Short = 6
Delay_Short = 0.05
Delay_Long = 0.1
Averaging_Samples = 10
Channel_S1 = 0
Channel_S2 = 1
Channel_S3 = 2
Radius = 0.0225 # meters
Dynamic_Viscosity = 8.90*pow(10,-4) # Pascal per second
Length = 0.165 # meters
Area = math.pi*pow(Radius,2)

#ADC_Readings = {"S1":{},"S2":{},"S3":{}}
Voltage_Readings = {"S1":{},"S2":{},"S3":{}}
Pressure_Readings = {"S1":{},"S2":{},"S3":{}}
Offsets = [0.498375,0.498375,0.498375,0.47695312500000003,0.47695312500000003]


# ------ Methods --------

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
        My_Coefficient = 0.03125
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
    
    # Make float times coeffcient
    Voltage = float(Raw_Data*My_Coefficient)
    
    # Return
    return Voltage

def Prepare_Data():
    # Fix data
    # Initialise lists
    #ADC_List = []
    Voltage_List = []
    Pressure_List = []
    
    # Make list of every read value
    for Sensor in Voltage_Readings:
        for Voltage in Voltage_Readings[Sensor]:
            Voltage_List.append(Voltage)
        for Pressure in Pressure_Readings[Sensor]:
            Pressure_List.append(Pressure)
        #for Data in ADC_Readings[Sensor]:
            #ADC_List.append(Data)
    # Remove duplicates
    Voltage_List = list(set(Voltage_List))
    Pressure_List = list(set(Pressure_List))
    #ADC_List = list(set(ADC_List))
    
    # Sort the lists
    Voltage_List.sort()
    Pressure_List.sort()
    #ADC_List.sort()
    
    return Voltage_List, Pressure_List #,ADC_List
    
def Print_Data(Voltage_List,Pressure_List): #ADC_List,
    # Initialize strings
    S1_String = ""
    S2_String = ""
    S3_String = ""
    
    # Go through all ADC values
    for i in range(len(Voltage_List)):
        
            # If in the readins for S1
            if Voltage_List[i] in Voltage_Readings["S1"]:
                S1_String += str(Voltage_Readings["S1"][Voltage_List[i]])
                S1_String += " "
            # Else if not present insert 0
            else:
                S1_String += str("0")
                S1_String += " "
                
            # Now the same check for S2
            if Voltage_List[i] in Voltage_Readings["S2"]:
                S2_String += str(Voltage_Readings["S2"][Voltage_List[i]])
                S2_String += " "
            else:
                S2_String += str("0")
                S2_String += " "
                
            # Now the same check for S3
            if Voltage_List[i] in Voltage_Readings["S3"]:
                S3_String += str(Voltage_Readings["S3"][Voltage_List[i]])
                S3_String += " "
            else:
                S3_String += str("0")
                S3_String += " "
        
    # Write data to file
    Test_File = open("Sensor_Test_ADC16","a")
    #Test_File.write("\nA: " + str(ADC_List))
    Test_File.write("\nV: " + str(Voltage_List))
    Test_File.write("\nP: " + str(Pressure_List))
    Test_File.write("\nS1: " + S1_String)
    Test_File.write("\nS2: " + S2_String)
    Test_File.write("\nS3: " + S3_String)
    
    Test_File.write("\n Better looking version: \n")
    
    # Write start line
    Test_File.write("\nVoltage:                 Pressure:                S1:   S2:   S3:\n")
    # Prepare and append strings
    for i in range(len(Voltage_List)):
        # Initialise sub strings
        #ADC_String = str(ADC_List[i])
        #ADC_String = ADC_String.ljust(Padding_Long,' ')
        
        V_String = str(Voltage_List[i])
        V_String = V_String.ljust(Padding_Long,' ')
        
        P_String = str(Pressure_List[i])
        P_String = P_String.ljust(Padding_Long,' ')
        
        if Voltage_List[i] in Voltage_Readings["S1"]:
            S1_String = str(Voltage_Readings["S1"][Voltage_List[i]])
            S1_String = S1_String.ljust(Padding_Short,' ')
        else:
            S1_String = str("0")
            S1_String = S1_String.ljust(Padding_Short,' ')
                
        if Voltage_List[i] in Voltage_Readings["S2"]:
            S2_String = str(Voltage_Readings["S2"][Voltage_List[i]])
            S2_String = S2_String.ljust(Padding_Short,' ')
        else:
            S2_String = str("0")
            S2_String = S2_String.ljust(Padding_Short,' ')
        
        if Voltage_List[i] in Voltage_Readings["S3"]:
            S3_String = str(Voltage_Readings["S3"][Voltage_List[i]])
            S3_String = S3_String.ljust(Padding_Short,' ')
        else:
            S3_String = str("0")
            S3_String = S3_String.ljust(Padding_Short,' ')
        S3_String = S3_String+"\n"
        
        # Complete string
        #Final_String = V_String+P_String+S1_String+S2_String+S3_String
        Final_String = V_String+P_String+S1_String+S2_String+S3_String
        # Write string
        Test_File.write(Final_String)
    # Close file
    Test_File.close()
    

def Get_ADC_Data():
    
    # Sensor 1
    # Set channel and mode to single
    Current_Sensor = "S1"
    Set_Channel(0)
    Set_Single()
    # Sleep for a bit
    time.sleep(Delay_Short)
    V1 = Read_Value()*pow(10,-3)
    #V1 = S1*Reference_V/Max_16_Bit
    P1 = (V1-Offsets[0])*Scaler
    time.sleep(Delay_Long)
    # Update stored data
    #if S1 in ADC_Readings[Current_Sensor]:
        #ADC_Readings[Current_Sensor][S1] += 1
    #else:
        #ADC_Readings[Current_Sensor][S1] = 1
        
    if V1 in Voltage_Readings[Current_Sensor]:
        Voltage_Readings[Current_Sensor][V1] += 1
    else:
        Voltage_Readings[Current_Sensor][V1] = 1
        
    if P1 in Pressure_Readings[Current_Sensor]:
        Pressure_Readings[Current_Sensor][P1] += 1
    else:
        Pressure_Readings[Current_Sensor][P1] = 1

    
    # Sensor 2
    # Set channel and mode to single
    Current_Sensor = "S2"
    Set_Channel(1)
    Set_Single()
    # Sleep for a bit
    time.sleep(Delay_Short)
    V2 = Read_Value()*pow(10,-3)
    #V2 = S2*Reference_V/Max_16_Bit
    P2 = (V2-Offsets[1])*Scaler
    time.sleep(Delay_Long)
    # Update stored data
    #if S2 in ADC_Readings[Current_Sensor]:
        #ADC_Readings[Current_Sensor][S2] += 1
    #else:
        #ADC_Readings[Current_Sensor][S2] = 1
        
    if V2 in Voltage_Readings[Current_Sensor]:
        Voltage_Readings[Current_Sensor][V2] += 1
    else:
        Voltage_Readings[Current_Sensor][V2] = 1
        
    if P2 in Pressure_Readings[Current_Sensor]:
        Pressure_Readings[Current_Sensor][P2] += 1
    else:
        Pressure_Readings[Current_Sensor][P2] = 1

    
    # Sensor 3
    # Set channel and mode to single
    Current_Sensor = "S3"
    Set_Channel(2)
    Set_Single()
    # Sleep for a bit
    time.sleep(Delay_Short)
    V3 = Read_Value()*pow(10,-3)
    #V3 = S3*Reference_V/Max_16_Bit
    P3 = (V3-Offsets[2])*Scaler
    
    time.sleep(Delay_Long)
    # Update stored data
    #if S3 in ADC_Readings[Current_Sensor]:
        #ADC_Readings[Current_Sensor][S3] += 1
    #else:
        #ADC_Readings[Current_Sensor][S3] = 1
        
    if V3 in Voltage_Readings[Current_Sensor]:
        Voltage_Readings[Current_Sensor][V3] += 1
    else:
        Voltage_Readings[Current_Sensor][V3] = 1
        
    if P3 in Pressure_Readings[Current_Sensor]:
        Pressure_Readings[Current_Sensor][P3] += 1
    else:
        Pressure_Readings[Current_Sensor][P3] = 1
    
    #print(ADC_List)
    #print(Voltage_List)
    #print(Pressure_List)

def Get_Reading(Sensor):
    # Set channel that matches the sensor
    if Sensor == "S1":
        Set_Channel(Channel_S1)
    elif Sensor == "S2":
        Set_Channel(Channel_S2)
    elif Sensor == "S3":
        Set_Channel(Channel_S3)
    else:
        print("Unknown sensor")
        return
    # Set mode to single
    Set_Single()
    
    # Sleep for a bit
    time.sleep(Delay_Short)
    
    # Define read voltage
    Voltage = Read_Value()*pow(10,-3) # Convert from mV to V
         
    # Return result
    return Voltage

def Update_Dict(Sensor,Voltage,Pressure):
    
    if Voltage in Voltage_Readings[Sensor]:
        Voltage_Readings[Sensor][Voltage] += 1
    else:
        Voltage_Readings[Sensor][Voltage] = 1
        
    if Pressure in Pressure_Readings[Sensor]:
        Pressure_Readings[Sensor][Pressure] += 1
    else:
        Pressure_Readings[Sensor][Pressure] = 1
        
def Calc_Velocity(Pressure_1, Pressure_2):
    # Calculate flow rate
    Flow_Rate = (math.pi*pow(Radius,4)*(Pressure_1-Pressure_2))/(8*Dynamic_Viscosity*Length)
    
    # Calculate Velocity
    Velocity = Flow_Rate/Area
    
    return Velocity

def Data_Handling():
    
    # Define work variable
    Voltage_Sum_S1 = 0
    Voltages_Sum_S2 = 0
    Voltages_Sum_S3 = 0
    
    # Average over given amount of samples
    for i in range(Averaging_Samples):
        # Add readings
        Voltage_Sum_S1 += Get_Reading("S1")
        Voltage_Sum_S2 += Get_Reading("S2")
        Voltage_Sum_S3 += Get_Reading("S3")
        
    # Calculate average
    Voltage_S1 = Voltage_Sum_S1/Averaing_Samples
    Voltage_S2 = Voltage_Sum_S2/Averaing_Samples
    Voltage_S3 = Voltage_Sum_S3/Averaing_Samples
    
    # Calculate pressure
    Pressure_S1 = (Voltage_S1-Offsets[Channel_S1])*Scaler
    Pressure_S2 = (Voltage_S2-Offsets[Channel_S2])*Scaler
    Pressure_S3 = (Voltage_S3-Offsets[Channel_S3])*Scaler
    
    # Update vectors:
    Update_Dict("S1",Voltage_S1,Pressure_S1)
    Update_Dict("S2",Voltage_S2,Pressure_S2)
    Update_Dict("S3",Voltage_S3,Pressure_S3)
   
def Sample_Test():
    # Get all wanted samples
    for i in range(Samples):
        # get data
        Data_Handling()
        
        # Samples left
        print(Samples-i)
    

# Main loop
while True:
    # Set address
    Set_Address(Address_48)
    # Set gain and input voltage range
    Set_Gain(Reg_Config_Range_6_144V)
    
    # Run test
    Sample_Test()
    
    # Prepare data
    Voltage_List,Pressure_List = Prepare_Data()
    
    # Print the results
    Print_Data(Voltage_List,Pressure_List)
    
    break
