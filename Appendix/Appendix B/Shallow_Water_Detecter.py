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
Samples = 500

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
#Dynamic_Viscosity = 8.90*pow(10,-4) # Pascal per second
Water_Viscosity = 0.01 # poise
Length = 0.165 # meters
Area = math.pi*pow(Radius,2)
Density_Water = 1000#998 # kg/m^3
Gravity = 9.815 # kg/m^3

Current_Height = 30/100 # 30 cm to meter

# Variables important to the sensors
Sensor_Offsets = [0.497375,0.4998125,0.4991875, 0.5059, 0.5053, 0.50053] # Previous 0.496 after that 0.486. New in regards to air
MVA_Raw_S1 = []
MVA_Raw_S2 = []
MVA_Raw_S3 = []
MVA_P_Average = []
Index = 0
MVA_Samples = 5 # Earlier 10

# Variables for motor control
Motor_1_En = 12 #26 #12
Motor_1_R = 21 #29 #21 
Motor_1_L = 20 #28 #20 

Motor_2_En = 13 #23 #13
Motor_2_R = 6 #22 #6 
Motor_2_L = 5 #21 #5
Button = 16
LED = 7

# Kalman variables
Kalman_Variance = [47.9709, 54.0366, 54.0366, 0.1, 0.1]
Process_Noice_Variance = 0.015
Old_State = [0,0,0,0,0]
Old_Variance = [0,0,0,0,0]

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
    Voltage = float(Raw_Data)*My_Coefficient #int(float(Raw_Data*My_Coefficient)
    
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
    GPIO.setup(Button,GPIO.IN)
    GPIO.setup(LED,GPIO.OUT)
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
    
    # Convert kPa to Pa
    Pressure_1 = Pressure_1*1000
    Pressure_2 = Pressure_2*1000
    
    # Calculate flow rate
    Flow_Rate = (math.pi*pow(Radius,4)*(Pressure_2-Pressure_1))/(8*Water_Viscosity*Length)
    
    # Calculate Velocity
    Velocity = Flow_Rate/Area

    
    
    return Velocity

# Method that calculates Height change from two pressure and velocity readings
def Calc_Height(Pressure_1,Pressure_2, Velocity_1, Velocity_2):
    global Current_Height
    # Convert kPa to Pa
    Pressure_1 = Pressure_1*1000
    Pressure_2 = Pressure_2*1000
    #print(Current_Height)

    # Use Bernoulli's principle to determine change in height
    # Calculate equation 1, using current height cm
    Value_Old = Pressure_2+(1/2)*Density_Water*pow(Velocity_2,2)+Density_Water*Gravity*0
    # Calculate part of equation 2 that does not include height
    Value_New = Pressure_1+(1/2)*Density_Water*pow(Velocity_1,2)
    # minus the two
    Height_Difference = Value_Old-Value_New
    # Devide with values timed onto height
    Height_Difference = Height_Difference/(Density_Water*Gravity)
    # Convert from meters to cm
    #Height_Difference = Height_Difference*100
    # Update height
    print(Height_Difference)
    Current_Height += Height_Difference
    
    return Height_Difference

#----------------------------- Kalman Filter ---------------------------------------------

# Step 0: Initialisation
def Kalman_Init(Reading, Sensor):
    # I choose the initialisation state as the first reading
    Init_State = Reading
    # Calculated in matlab after a 500 sample air test 
    if Sensor == "S1":
        Init_Variance = Kalman_Variance[0]
    elif Sensor == "S2":
        Init_Variance = Kalman_Variance[1]
    elif Sensor == "S3":
        Init_Variance = Kalman_Variance[2]
    elif Sensor == "P":
        Init_Variance = Kalman_Variance[0]
        
    return [Init_State,Init_Variance]

# Step 1: Measurement
def Kalman_Measurement(Sensor):
    # Get readings
    Readings = Get_Reading(Sensor)
    
    return Readings
    

# Step 2: Update
def Kalman_Update(Variance, Variance_Old, State, State_Old):
    
    # Calculate Kalman Gain
    Kalman_Gain = Variance_Old/(Variance_Old+Variance)
    
    # Estimate current state:
    Current_State_Estimate = State_Old + Kalman_Gain*(State-State_Old)
    
    # Update Variance estimate
    Current_Variance_Estimate = (1-Kalman_Gain)*Variance_Old
    
    return [Current_State_Estimate, Current_Variance_Estimate, Kalman_Gain]

# Step 3: Predict
def Kalman_Predict(Current_State_Estimate, Current_Variance_Estimate):
    
    # I assume constant dynamics
    State_Estimate_New = Current_State_Estimate
    
    Variance_Estimate_New = Current_Variance_Estimate + Process_Noice_Variance
    
    return [State_Estimate_New, Variance_Estimate_New]

def Kalman(First, S1_Reading, S2_Reading, S3_Reading):
    
    # Get readings
    #S1_Reading = Kalman_Measurement("S1")
    #S2_Reading = Kalman_Measurement("S2")
    #S3_Reading = Kalman_Measurement("S3")
    
    # If first iteration
    if First == True:
        # Initialise estimate
        S1_Update = Kalman_Init(S1_Reading, "S1")
        S2_Update = Kalman_Init(S2_Reading, "S2")
        S3_Update = Kalman_Init(S3_Reading, "S3")
        
        # Predict
        S1_Predict = Kalman_Predict(S1_Update[0], S1_Update[1])
        S2_Predict = Kalman_Predict(S2_Update[0], S2_Update[1])
        S3_Predict = Kalman_Predict(S3_Update[0], S3_Update[1])
    # If not first iteration
    else:
        # Update
        S1_Update = Kalman_Update(Kalman_Variance[0], Old_Variance[0], S1_Reading, Old_State[0])
        S2_Update = Kalman_Update(Kalman_Variance[1], Old_Variance[1], S2_Reading, Old_State[1])
        S3_Update = Kalman_Update(Kalman_Variance[2], Old_Variance[2], S3_Reading, Old_State[2])
        
        # Predict
        S1_Predict = Kalman_Predict(S1_Update[0], S1_Update[1])
        S2_Predict = Kalman_Predict(S2_Update[0], S2_Update[1])
        S3_Predict = Kalman_Predict(S3_Update[0], S3_Update[1])
        
    # Update Old values
    Old_State[0] = S1_Predict[0]
    Old_State[1] = S2_Predict[0]
    Old_State[2] = S3_Predict[0]
    
    Old_Variance[0] = S1_Predict[1]
    Old_Variance[1] = S2_Predict[1]
    Old_Variance[2] = S3_Predict[1]
    
    # Return Estimate state
    return [S1_Update[0],S2_Update[0],S2_Update[0]]

def Kalman_Single(First,Reading, Type):

    # If first iteration
    if First == True:
        # Initialise estimate
        Update = Kalman_Init(Reading, Type)
        
        # Predict
        Predict = Kalman_Predict(Update[0], Update[1])

    # If not first iteration
    else:
        # Update
        if Type == "P":
            Update = Kalman_Update(Kalman_Variance[3], Old_Variance[3], Reading, Old_State[3])
    
        # Predict
        Predict = Kalman_Predict(Update[0], Update[1])
        
    # Update Old values
    if Type == "P":
        Old_State[3] = Predict[0]
    
        Old_Variance[3] = Predict[1]
    
    # Return Estimate state
    return Update[0]

# ---------------------------- Other Data handling Methods ----------------------------------

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
    if len(MVA_Raw_S1) < MVA_Samples:
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
    
    #Kalman
    Kalman_Raw = Kalman(True, S1_Average, S2_Average, S3_Average)

    
    # Calculate Voltage for average readings
    S1_V_Average = Get_Voltage(Kalman_Raw[0])
    S2_V_Average = Get_Voltage(Kalman_Raw[1])
    #S3_V_Average = Get_Voltage(S3_Average) Old version
    S3_V_Average = Get_Voltage(Kalman_Raw[2])
    
    # Calculate Pressure for average readings
    S1_P_Average = (S1_V_Average-Sensor_Offsets[3])*Scaler
    S2_P_Average = (S2_V_Average-Sensor_Offsets[4])*Scaler
    S3_P_Average = (S3_V_Average-Sensor_Offsets[5])*Scaler
    
    # Collect into dict
    S1_Average_Readings = {"Raw":Kalman_Raw[0],"Voltage":S1_V_Average,"Pressure":S1_P_Average}
    S2_Average_Readings = {"Raw":Kalman_Raw[1],"Voltage":S2_V_Average,"Pressure":S2_P_Average}
    S3_Average_Readings = {"Raw":Kalman_Raw[2],"Voltage":S3_V_Average,"Pressure":S3_P_Average}
    
    # Moving average over average pressure
    if len(MVA_P_Average) < MVA_Samples:
        MVA_P_Average.append((S1_P_Average+S2_P_Average+S3_P_Average)/3)
    else:
        MVA_P_Average[Index] = ((S1_P_Average+S2_P_Average+S3_P_Average)/3)
    # Calc result
    P_Average = 0
    for i in range(len(MVA_P_Average)):
        P_Average += MVA_P_Average[i]
        
    P_Average = P_Average/len(MVA_P_Average)
    
    # Update index
    if Index == MVA_Samples-1:
        Index = 0
    else:
        Index += 1
    
    return S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings, P_Average

def Print_Data(S1_Readings, S2_Readings, S3_Readings, Heights, Pressures):
    
    # Write data to file
    Test_File = open(r"Test_Where.txt","a")
    time.sleep(Delay_Short)
    
    # Write Raw Values
    Test_File.write("\nRaw Data: ")
    time.sleep(Delay_Short)
    Test_File.write("\nS1 Raw Data: " + str(S1_Readings["Raw"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS2 Raw Data: " + str(S2_Readings["Raw"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS3 Raw Data: " + str(S3_Readings["Raw"]))
    time.sleep(Delay_Short)
    
    # Write Voltage Values
    Test_File.write("\nVoltage Data: ")
    time.sleep(Delay_Short)
    Test_File.write("\nS1 Voltage Data: " + str(S1_Readings["Voltage"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS2 Voltage Data: " + str(S2_Readings["Voltage"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS3 Voltage Data: " + str(S3_Readings["Voltage"]))
    time.sleep(Delay_Short)
    
    # Write Pressure Values
    Test_File.write("\nPressure Data: ")
    time.sleep(Delay_Short)
    Test_File.write("\nS1 Pressure Data: " + str(S1_Readings["Pressure"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS2 Pressure Data: " + str(S2_Readings["Pressure"]))
    time.sleep(Delay_Short)
    Test_File.write("\nS3 Pressure Data: " + str(S3_Readings["Pressure"]))
    time.sleep(Delay_Short)
    
    # Write calculated heights
    Test_File.write("\nCalculated Heights: ")
    time.sleep(Delay_Short)
    Test_File.write(str(Heights))
    time.sleep(Delay_Short)
    
    # Write calculated pressures
    Test_File.write("\nCalculated pressures: ")
    time.sleep(Delay_Short)
    Test_File.write(str(Pressures))
    time.sleep(Delay_Short)
    
    # Close file
    Test_File.close()
    time.sleep(Delay_Short)
    
def Sample_Test():
    # Setup work dicts
    S1_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S2_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S3_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    Heights = []
    Pressures = []
    Velocity = 0
    Velocity_Old = 0
    Direction_Counter = 0
    Current_Direction = "Forward"
    
    # Get desired amount of data
    for i in range(Samples):
        # Get Data
        S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings, P_Average = Moving_Average_Filter()
        
        # Kalman test moved to MVA function
#         if i == 0:
#             Kalman_Raw = Kalman(True, S1_Average_Readings["Raw"], S2_Average_Readings["Raw"], S3_Average_Readings["Raw"])
#             Kalman_V = Kalman(True, S1_Average_Readings["Voltage"], S2_Average_Readings["Voltage"], S3_Average_Readings["Voltage"])
#             Kalman_P = Kalman(True, S1_Average_Readings["Pressure"], S2_Average_Readings["Pressure"], S3_Average_Readings["Pressure"])
#             Kalman_P_Average = Kalman_Single(True,P_Average, "P")
#         else:
#             Kalman_Raw = Kalman(False, S1_Average_Readings["Raw"], S2_Average_Readings["Raw"], S3_Average_Readings["Raw"])
#             Kalman_V = Kalman(False, S1_Average_Readings["Voltage"], S2_Average_Readings["Voltage"], S3_Average_Readings["Voltage"])
#             Kalman_P = Kalman(False, S1_Average_Readings["Pressure"], S2_Average_Readings["Pressure"], S3_Average_Readings["Pressure"])
#             Kalman_P_Average = Kalman_Single(False,P_Average, "P")
        
        # Update work dicts
        S1_Readings_T["Raw"].append(S1_Average_Readings["Raw"])
        #S1_Readings_T["Raw"].append(Kalman_Raw[0])
        S1_Readings_T["Voltage"].append(S1_Average_Readings["Voltage"])
        #S1_Readings_T["Voltage"].append(Kalman_V[0])
        S1_Readings_T["Pressure"].append(S1_Average_Readings["Pressure"])
        #S1_Readings_T["Pressure"].append(Kalman_P[0])
        
        S2_Readings_T["Raw"].append(S2_Average_Readings["Raw"])
        #S2_Readings_T["Raw"].append(Kalman_Raw[1])
        S2_Readings_T["Voltage"].append(S2_Average_Readings["Voltage"])
        #S2_Readings_T["Voltage"].append(Kalman_V[1])
        S2_Readings_T["Pressure"].append(S2_Average_Readings["Pressure"])
        #S2_Readings_T["Pressure"].append(Kalman_P[1])
        
        S3_Readings_T["Raw"].append(S3_Average_Readings["Raw"])
        #S3_Readings_T["Raw"].append(Kalman_Raw[2])
        S3_Readings_T["Voltage"].append(S3_Average_Readings["Voltage"])
        #S3_Readings_T["Voltage"].append(Kalman_V[2])
        S3_Readings_T["Pressure"].append(S3_Average_Readings["Pressure"])
        #S3_Readings_T["Pressure"].append(Kalman_P[2])
        
        Pressures.append(P_Average)
        #Pressures.append(Kalman_P_Average)
        
        # Calculate velocity
        Velocity_Old = Velocity
        Velocity = Calc_Velocity(S1_Average_Readings["Pressure"], S3_Average_Readings["Pressure"])
        #Velocity = Calc_Velocity(Kalman_P[0], Kalman_P[2])
        
        # If more than one reading calculate height
        if len(S3_Readings_T["Raw"]) > 1:
            Heights.append(Calc_Height(S2_Readings_T["Pressure"][-1],S2_Readings_T["Pressure"][-2], Velocity, Velocity_Old))
        
        #print(Current_Height)
        # print current sample
        #print(i)
#         Direction_Counter += 1
#         if Direction_Counter == 50:
#             Direction_Counter = 0
#             if Current_Direction == "Forward":
#                 Move_Backwards()
#                 Current_Direction = "Backwards"
#             elif Current_Direction == "Backwards":
#                 Move_Forward()
#                 Current_Direction = "Forward"
#         elif abs(Heights[-1]) > 1:
#             if Current_Direction == "Forward":
#                 Move_Backwards()
#                 Current_Direction = "Backwards"
#             elif Current_Direction == "Backwards":
#                 Move_Forward()
#                 Current_Direction = "Forward"
            
    # print results to file
    Print_Data(S1_Readings_T, S2_Readings_T, S3_Readings_T, Heights, Pressures)
    
# --------------------------------- Main ----------------------------------------------

# while GPIO.input(Button) == GPIO.LOW:
#     print("Waiting for button")

# Set address
Set_Address(Address_48)
# Set gain and input voltage range
Set_Gain(Reg_Config_Range_1_024V)
# Setup GPIO
GPIO_Setup()

GPIO.output(LED, GPIO.HIGH)

while GPIO.input(Button) == GPIO.LOW:
    print("Waiting for button")
# Run test
#Move_Forward()

Sample_Test()

GPIO.output(LED, GPIO.LOW)

#Move_Stop()