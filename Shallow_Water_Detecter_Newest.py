# -------------------- A Collected File with all necessary code ----------------------

# ---------------------------- Imports------------------------------------------------
import smbus
import time
import math
import RPi.GPIO as GPIO
import numpy as np
import scipy
print(scipy.__version__)
from scipy.fftpack import fft, rfft, fftfreq
#from scipy.fftpack import fftfreq, rfftfreq
from scipy.signal import firwin, remez, kaiser_atten, kaiser_beta, freqz, detrend, lfilter
#from scipy.fft import fft, fftfreq
#from scipy.signal import fft
#import scipy.fft as FFT

import matplotlib.pyplot as plt
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
Samples = 1000
Proces_Samples = 10

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
Sensor_Offsets_Kal = [0.49940625,0.4968125,0.4968125]
Sensor_Offsets_MVA = [0.49903125000000004,0.4968125,0.497]
Sensor_Offsets_MVA_3 = [0.49903125000000004,0.4968125,0.497]
Sensor_Offsets_MVA_8 = [0.49903125000000004,0.4968125,0.497]
Sensor_Offsets_MVA_10 = [0.49903125000000004,0.4968125,0.497]
Sensor_Offsets_Gen = [0.49403125000000003, 0.49078125, 0.49315625]

MVA_Raw_S1 = []
MVA_Raw_S2 = []
MVA_Raw_S3 = []

MVA_Raw_S1_3 = []
MVA_Raw_S2_3 = []
MVA_Raw_S3_3 = []

MVA_Raw_S1_8 = []
MVA_Raw_S2_8 = []
MVA_Raw_S3_8 = []

MVA_Raw_S1_10 = []
MVA_Raw_S2_10 = []
MVA_Raw_S3_10 = []

MVA_P_Average = []
Index = 0
Index_3 = 0
Index_8 = 0
Index_10 = 0
MVA_Samples = 5 # Earlier 10
MVA_Samples_3 = 3
MVA_Samples_8 = 8
MVA_Samples_10 = 10

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

Old_State_015 = [0,0,0,0,0]
Old_Variance_015 = [0,0,0,0,0]

Old_State_00015 = [0,0,0,0,0]
Old_Variance_00015 = [0,0,0,0,0]

Old_State_000015 = [0,0,0,0,0]
Old_Variance_000015 = [0,0,0,0,0]

# New Kal
Old_State_S1 = 0
Old_State_S2 = 0
Old_State_S3 = 0

Old_Var_S1 = 0
Old_Var_S2 = 0
Old_Var_S3 = 0

# FIR
# Sample_Rate = 860 # Samples per second
# Number_Samples = 0 # 0 to start off with
# Nyq_Rate = Sample_Rate/2 # Nyquist rate
# Samples = []
# Width = 5/Nyq_Rate # 5 Hz width
# Ripple_DB = 60 # Desired attenuation in stop band in decibel
# N,Beta = kaiserord(Ripple_DB, Width) # Computing order and Kaiser parameter
# Cutoff_Hz = 10 # Cuttoff frequency
# Taps = firwin(N,Cutoff_Hz/Nyq_Rate, window('kaiser',Beta)) # Kaiser window to create lowpas Fir Filter
# Filtered_Samples = lfilter(Taps,1.0,Samples)

#----------------------------------FIR Implementation



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
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_0 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_860SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 1:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_1 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_860SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 2:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_2 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_860SPS | Reg_Cofig_Alert_None]
    elif My_Channel == 3:
        Config_Reg = [Reg_Config_Single | Reg_Config_Single_3 | My_Gain | Reg_Config_Mode_Continuous, Reg_Config_860SPS | Reg_Cofig_Alert_None]
    
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
    Current_Height += Height_Difference
    
    return Height_Difference
#---------------------------Kalman Ne

def Kal_Predict(a,x_old,p_old,q):
    # a = State transition
    # x_old = last state
    # p_old = error (covariance)
    # q = transition error
    
    x = a*x_old # predict state
    p = a*p_old*a*q # predict error
    
    return [x,p]

def Kal_Update(z,h,x,p,r):
    # z = Meassurement
    # h = state-to-measurement transformation
    # x = state preducted
    # p = error predicted
    # r = measurement error
    
    y = z-h*x # Difference between measure and predict
    
    kg = p * h/(h*p*h+r) # Kalman gain
    
    x = x+kg*y # updated state
    
    p = (1-kg*h)*p # Updated error

def Kal(S1,S2,S3,First):
    h = 1 # Since we estimate what we meassure
    
    if First == True:
        x1 = S1
        x2 = S2
        x3 = S3
        
        #p1 =
        #p2 =
        #p3 =
    
    #else:
    

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
def Kalman_Predict(Current_State_Estimate, Current_Variance_Estimate, Process_Noice_V):
    
    # I assume constant dynamics
    State_Estimate_New = Current_State_Estimate
    
    Variance_Estimate_New = Current_Variance_Estimate + Process_Noice_V
    
    return [State_Estimate_New, Variance_Estimate_New]

def Kalman(First, S1_Reading, S2_Reading, S3_Reading, Process_Noice_V):
    
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
        S1_Predict = Kalman_Predict(S1_Update[0], S1_Update[1],Process_Noice_V)
        S2_Predict = Kalman_Predict(S2_Update[0], S2_Update[1],Process_Noice_V)
        S3_Predict = Kalman_Predict(S3_Update[0], S3_Update[1],Process_Noice_V)
    # If not first iteration
    else:
        # Update
        if Process_Noice_V == 0.015:
            S1_Update = Kalman_Update(Kalman_Variance[0], Old_Variance[0], S1_Reading, Old_State[0])
            S2_Update = Kalman_Update(Kalman_Variance[1], Old_Variance[1], S2_Reading, Old_State[1])
            S3_Update = Kalman_Update(Kalman_Variance[2], Old_Variance[2], S3_Reading, Old_State[2])
        elif Process_Noice_V == 0.15:
            S1_Update = Kalman_Update(Kalman_Variance[0], Old_Variance_015[0], S1_Reading, Old_State_015[0])
            S2_Update = Kalman_Update(Kalman_Variance[1], Old_Variance_015[1], S2_Reading, Old_State_015[1])
            S3_Update = Kalman_Update(Kalman_Variance[2], Old_Variance_015[2], S3_Reading, Old_State_015[2])
        elif Process_Noice_V == 0.0015:
            S1_Update = Kalman_Update(Kalman_Variance[0], Old_Variance_00015[0], S1_Reading, Old_State_00015[0])
            S2_Update = Kalman_Update(Kalman_Variance[1], Old_Variance_00015[1], S2_Reading, Old_State_00015[1])
            S3_Update = Kalman_Update(Kalman_Variance[2], Old_Variance_00015[2], S3_Reading, Old_State_00015[2])
        elif Process_Noice_V == 0.00015:
            S1_Update = Kalman_Update(Kalman_Variance[0], Old_Variance_000015[0], S1_Reading, Old_State_000015[0])
            S2_Update = Kalman_Update(Kalman_Variance[1], Old_Variance_000015[1], S2_Reading, Old_State_000015[1])
            S3_Update = Kalman_Update(Kalman_Variance[2], Old_Variance_000015[2], S3_Reading, Old_State_000015[2])
            
        # Predict
        S1_Predict = Kalman_Predict(S1_Update[0], S1_Update[1],Process_Noice_V)
        S2_Predict = Kalman_Predict(S2_Update[0], S2_Update[1],Process_Noice_V)
        S3_Predict = Kalman_Predict(S3_Update[0], S3_Update[1],Process_Noice_V)
        
    # Update Old values
    if Process_Noice_V == 0.015:
        Old_State[0] = S1_Predict[0]
        Old_State[1] = S2_Predict[0]
        Old_State[2] = S3_Predict[0]
    
        Old_Variance[0] = S1_Predict[1]
        Old_Variance[1] = S2_Predict[1]
        Old_Variance[2] = S3_Predict[1]
    elif Process_Noice_V == 0.15:
        Old_State_015[0] = S1_Predict[0]
        Old_State_015[1] = S2_Predict[0]
        Old_State_015[2] = S3_Predict[0]
    
        Old_Variance_015[0] = S1_Predict[1]
        Old_Variance_015[1] = S2_Predict[1]
        Old_Variance_015[2] = S3_Predict[1]
    
    elif Process_Noice_V == 0.0015:
        Old_State_00015[0] = S1_Predict[0]
        Old_State_00015[1] = S2_Predict[0]
        Old_State_00015[2] = S3_Predict[0]
    
        Old_Variance_00015[0] = S1_Predict[1]
        Old_Variance_00015[1] = S2_Predict[1]
        Old_Variance_00015[2] = S3_Predict[1]
        
    elif Process_Noice_V == 0.00015:
        Old_State_000015[0] = S1_Predict[0]
        Old_State_000015[1] = S2_Predict[0]
        Old_State_000015[2] = S3_Predict[0]
    
        Old_Variance_000015[0] = S1_Predict[1]
        Old_Variance_000015[1] = S2_Predict[1]
        Old_Variance_000015[2] = S3_Predict[1] 
    
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
    #Voltage = Get_Voltage(Raw_Data)
    
    # Define Pressure
    #Pressure = (Voltage-Offset)*Scaler
    
    # Collect Data into Dict
    #Result = {"Raw":Raw_Data,"Voltage":Voltage,"Pressure":Pressure}
    #Result = {"Raw":Raw_Data}
    Result = Raw_Data
                  
    # Long sleep
    #time.sleep(Delay_Long)
         
    # Return result
    return Result

def Moving_Average_Filter_Updated(S1_Raw,S2_Raw,S3_Raw):
    global Index

    # Insert newest values
    if len(MVA_Raw_S1) < MVA_Samples:
        MVA_Raw_S1.append(S1_Raw)
        MVA_Raw_S2.append(S2_Raw)
        MVA_Raw_S3.append(S3_Raw)
        
    else:
        MVA_Raw_S1[Index] = S1_Raw
        MVA_Raw_S2[Index] = S2_Raw
        MVA_Raw_S3[Index] = S3_Raw
        
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
    
    # Update index
    if Index == MVA_Samples-1:
        Index = 0
    else:
        Index += 1
    
    return S1_Average, S2_Average, S3_Average

def Moving_Average_Filter_Updated_3(S1_Raw,S2_Raw,S3_Raw):
    global Index_3

    # Insert newest values
    if len(MVA_Raw_S1_3) < MVA_Samples_3:
        MVA_Raw_S1_3.append(S1_Raw)
        MVA_Raw_S2_3.append(S2_Raw)
        MVA_Raw_S3_3.append(S3_Raw)
        
    else:
        MVA_Raw_S1_3[Index_3] = S1_Raw
        MVA_Raw_S2_3[Index_3] = S2_Raw
        MVA_Raw_S3_3[Index_3] = S3_Raw
        
    # Calculate Average
    S1_Average = 0
    S2_Average = 0
    S3_Average = 0
    
    for i in range(len(MVA_Raw_S1_3)):
        S1_Average += MVA_Raw_S1_3[i]
        S2_Average += MVA_Raw_S2_3[i]
        S3_Average += MVA_Raw_S3_3[i]
        
    S1_Average = S1_Average/len(MVA_Raw_S1_3)
    S2_Average = S2_Average/len(MVA_Raw_S2_3)
    S3_Average = S3_Average/len(MVA_Raw_S3_3)
    
    # Update index
    if Index_3 == MVA_Samples_3-1:
        Index_3 = 0
    else:
        Index_3 += 1
    
    return S1_Average, S2_Average, S3_Average

def Moving_Average_Filter_Updated_8(S1_Raw,S2_Raw,S3_Raw):
    global Index_8

    # Insert newest values
    if len(MVA_Raw_S1_8) < MVA_Samples_8:
        MVA_Raw_S1_8.append(S1_Raw)
        MVA_Raw_S2_8.append(S2_Raw)
        MVA_Raw_S3_8.append(S3_Raw)
        
    else:
        MVA_Raw_S1_8[Index_8] = S1_Raw
        MVA_Raw_S2_8[Index_8] = S2_Raw
        MVA_Raw_S3_8[Index_8] = S3_Raw
        
    # Calculate Average
    S1_Average = 0
    S2_Average = 0
    S3_Average = 0
    
    for i in range(len(MVA_Raw_S1_8)):
        S1_Average += MVA_Raw_S1_8[i]
        S2_Average += MVA_Raw_S2_8[i]
        S3_Average += MVA_Raw_S3_8[i]
        
    S1_Average = S1_Average/len(MVA_Raw_S1_8)
    S2_Average = S2_Average/len(MVA_Raw_S2_8)
    S3_Average = S3_Average/len(MVA_Raw_S3_8)
    
    # Update index
    if Index_8 == MVA_Samples_8-1:
        Index_8 = 0
    else:
        Index_8 += 1
    
    return S1_Average, S2_Average, S3_Average

def Moving_Average_Filter_Updated_10(S1_Raw,S2_Raw,S3_Raw):
    global Index_10

    # Insert newest values
    if len(MVA_Raw_S1_10) < MVA_Samples_10:
        MVA_Raw_S1_10.append(S1_Raw)
        MVA_Raw_S2_10.append(S2_Raw)
        MVA_Raw_S3_10.append(S3_Raw)
        
    else:
        MVA_Raw_S1_10[Index_10] = S1_Raw
        MVA_Raw_S2_10[Index_10] = S2_Raw
        MVA_Raw_S3_10[Index_10] = S3_Raw
        
    # Calculate Average
    S1_Average = 0
    S2_Average = 0
    S3_Average = 0
    
    for i in range(len(MVA_Raw_S1_10)):
        S1_Average += MVA_Raw_S1_10[i]
        S2_Average += MVA_Raw_S2_10[i]
        S3_Average += MVA_Raw_S3_10[i]
    
    S1_Average = S1_Average/len(MVA_Raw_S1_10)
    S2_Average = S2_Average/len(MVA_Raw_S2_10)
    S3_Average = S3_Average/len(MVA_Raw_S3_10)
    
    # Update index
    if Index_10 == MVA_Samples_10-1:
        Index_10 = 0
    else:
        Index_10 += 1
    
    return S1_Average, S2_Average, S3_Average

def Moving_Average_Filter():
    global Index
    
    # Get readings
    S1_Readings = Get_Reading("S1")
    S2_Readings = Get_Reading("S2")
    S3_Readings = Get_Reading("S3")
    
    #Kalman if before MVA
    #Kalman_Raw = Kalman(True, S1_Readings["Raw"], S2_Readings["Raw"], S3_Readings["Raw"])

    
    # Insert newest values
    if len(MVA_Raw_S1) < MVA_Samples:
        MVA_Raw_S1.append(S1_Readings["Raw"])
        MVA_Raw_S2.append(S2_Readings["Raw"])
        MVA_Raw_S3.append(S3_Readings["Raw"])
        #MVA_Raw_S1.append(Kalman_Raw[0])
        #MVA_Raw_S2.append(Kalman_Raw[1])
        #MVA_Raw_S3.append(Kalman_Raw[2])
    else:
        MVA_Raw_S1[Index] = S1_Readings["Raw"]
        MVA_Raw_S2[Index] = S2_Readings["Raw"]
        MVA_Raw_S3[Index] = S3_Readings["Raw"]
        #MVA_Raw_S1[Index] = Kalman_Raw[0]
        #MVA_Raw_S2[Index] = Kalman_Raw[1]
        #MVA_Raw_S3[Index] = Kalman_Raw[2]
    
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
    
    #Kalman if after MVA
    #Kalman_Raw = Kalman(True, S1_Average, S2_Average, S3_Average)

    
    # Calculate Voltage for average readings
    #S1_V_Average = Get_Voltage(Kalman_Raw[0])
    #S2_V_Average = Get_Voltage(Kalman_Raw[1])
    #S3_V_Average = Get_Voltage(Kalman_Raw[2])
    #S1_V_Average = Get_Voltage(S1_Average)
    #S2_V_Average = Get_Voltage(S2_Average)
    #S3_V_Average = Get_Voltage(S3_Average)
    
    # Calculate Pressure for average readings
    #S1_P_Average = (S1_V_Average-Sensor_Offsets[3])*Scaler
    #S2_P_Average = (S2_V_Average-Sensor_Offsets[4])*Scaler
    #S3_P_Average = (S3_V_Average-Sensor_Offsets[5])*Scaler
    
    # Collect into dict
    S1_Average_Readings = {"Raw":Kalman_Raw[0]}
    S2_Average_Readings = {"Raw":Kalman_Raw[1]}
    S3_Average_Readings = {"Raw":Kalman_Raw[2]}
    #S1_Average_Readings = {"Raw":Kalman_Raw[0],"Voltage":S1_V_Average,"Pressure":S1_P_Average}
    #S2_Average_Readings = {"Raw":Kalman_Raw[1],"Voltage":S2_V_Average,"Pressure":S2_P_Average}
    #S3_Average_Readings = {"Raw":Kalman_Raw[2],"Voltage":S3_V_Average,"Pressure":S3_P_Average}
    #S1_Average_Readings = {"Raw":S1_Average,"Voltage":S1_V_Average,"Pressure":S1_P_Average}
    #S2_Average_Readings = {"Raw":S2_Average,"Voltage":S2_V_Average,"Pressure":S2_P_Average}
    #S3_Average_Readings = {"Raw":S3_Average,"Voltage":S3_V_Average,"Pressure":S3_P_Average}
    
    
    # Moving average over average pressure
    #if len(MVA_P_Average) < MVA_Samples:
    #    MVA_P_Average.append((S1_P_Average+S2_P_Average+S3_P_Average)/3)
    #else:
    #    MVA_P_Average[Index] = ((S1_P_Average+S2_P_Average+S3_P_Average)/3)
    # Calc result
    #P_Average = 0
    #for i in range(len(MVA_P_Average)):
    #    P_Average += MVA_P_Average[i]
        
    #P_Average = P_Average/len(MVA_P_Average)
    
    # Update index
    if Index == MVA_Samples-1:
        Index = 0
    else:
        Index += 1
    
    return S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings#, P_Average

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

def Signal_Processing(First):
    
    # Setup work vector
    Processing_Data_S1 = []
    Processing_Data_S2 = []
    Processing_Data_S3 = []
    
    # Get desired amount of data via MVA
    for i in range(Proces_Samples):
        
        # Get raw readings
        S1_Reading = Get_Reading("S1")
        S2_Reading = Get_Reading("S2")
        S3_Reading = Get_Reading("S3")
        
        # Get value through Moving Average Filter
        S1_MVA, S2_MVA, S3_MVA = Moving_Average_Filter_Updated(S1_Reading, S2_Reading, S3_Reading)
        
        # Kalman filter
        if First == True and i == 0:
            Kalman_Raw = Kalman(True, S1_MVA, S2_MVA, S3_MVA)
        else:
            Kalman_Raw = Kalman(False, S1_MVA, S2_MVA, S3_MVA)
            
        # Add results to data sets
        Processing_Data_S1.append(int(Kalman_Raw[0]))
        Processing_Data_S2.append(int(Kalman_Raw[1]))
        Processing_Data_S3.append(int(Kalman_Raw[2]))
        
    # Return result
    return Processing_Data_S1, Processing_Data_S2, Processing_Data_S3

def Detect_Shallow_Water():
    # Setup work dicts
    S1_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S2_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    S3_Readings_T = {"Raw":[], "Voltage":[], "Pressure":[]}
    
    # Run through all samples
    for i in range(Samples):
        
        # If first iteration
        if i == 0:
            S1_Data, S2_Data, S3_Data = Signal_Processing(True)
        else:
            S1_Data, S2_Data, S3_Data = Signal_Processing(False)
        
        # Calculate Voltages and pressures
        for index in range(S1_Data):
            S1_Current = S1_Data[index]
            S2_Current = S2_Data[index]
            S3_Current = S3_Data[index]
            
            # Velocities
            
            
            S1_Readings_T["Voltage"].append(Get_Voltage(S1_Current))
            S2_Readings_T["Voltage"].append(Get_Voltage(S2_Current))
            S3_Readings_T["Voltage"].append(Get_Voltage(S3_Current))
        
        # Calculate Pressures
        for index in range(S1_Data):
            S1_Readings_T["Voltage"].append((S1_V-Sensor_Offsets[3])*Scaler)
            S2_Readings_T["Voltage"].append((S2_V_Average-Sensor_Offsets[4])*Scaler)
            S3_Readings_T["Voltage"].append((S3_V_Average-Sensor_Offsets[5])*Scaler)
        
        
    # Convert to numpy array
    #NP_S1 = np.array(Processing_Data_S1)
    #NP_S2 = np.array(Processing_Data_S2)
    #NP_S3 = np.array(Processing_Data_S3)
    
    # Detrend
    #NP_S1 = signal.detrend(NP_S1)
    #NP_S2 = signal.detrend(NP_S2)
    #NP_S3 = signal.detrend(NP_S3)
    
    # Fourier transform data
    #FFT_S1 = np.abs(np.fft.fft(NP_S1,axis=0))
    #FFT_S2 = np.abs(np.fft.fft(NP_S2,axis=0))
    #FFT_S3 = np.abs(np.fft.fft(NP_S1,axis=0))
    
    # Calc magnitude
    #FFT_S1_Mag = abs(FFT_S1[:,0])
    #FFT_S2_Mag = abs(FFT_S2[:,0])
    #FFT_S3_Mag = abs(FFT_S3[:,0])
    
    # Normalize with N/2
    #N = len(FFT_S1)
    #Norm = N/2
    
    #FFT_S1 = np.abs(FFT_S1)/Norm
    #FFT_S2 = np.abs(FFT_S2)/Norm
    #FFT_S3 = np.abs(FFT_S3)/Norm
    
    # Test
    #plt.plot(FFT_S1_Mag)
    #plt.show()
    
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
        S1_Readings, S2_Readings, S3_Readings, S1_Average_Readings, S2_Average_Readings, S3_Average_Readings = Moving_Average_Filter() #, P_Average
        
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

def Comparison_Test():
    
    RAW_S1 = []
    RAW_S2 = []
    RAW_S3 = []
    
    V1_Raw = []
    V2_Raw = []
    V3_Raw = []
    
    V1_MVA = []
    V2_MVA = []
    V3_MVA = []
    
    V1_MVA_3 = []
    V2_MVA_3 = []
    V3_MVA_3 = []
    
    V1_MVA_8 = []
    V2_MVA_8 = []
    V3_MVA_8 = []
    
    V1_MVA_10 = []
    V2_MVA_10 = []
    V3_MVA_10 = []
    
    V1_Kalman = []
    V2_Kalman = []
    V3_Kalman = []
    V1_Kalman_015 = []
    V2_Kalman_015 = []
    V3_Kalman_015 = []
    V1_Kalman_00015 = []
    V2_Kalman_00015 = []
    V3_Kalman_00015 = []
    V1_Kalman_000015 = []
    V2_Kalman_000015 = []
    V3_Kalman_000015 = []
    
    
    P1_MVA = []
    P2_MVA = []
    P3_MVA = []
    
    P1_MVA_3 = []
    P2_MVA_3 = []
    P3_MVA_3 = []
    
    P1_MVA_8 = []
    P2_MVA_8 = []
    P3_MVA_8 = []
    
    P1_MVA_10 = []
    P2_MVA_10 = []
    P3_MVA_10 = []
    
    P1_Kalman = []
    P2_Kalman = []
    P3_Kalman = []
    P1_Kalman_015 = []
    P2_Kalman_015 = []
    P3_Kalman_015 = []
    P1_Kalman_00015 = []
    P2_Kalman_00015 = []
    P3_Kalman_00015 = []
    P1_Kalman_000015 = []
    P2_Kalman_000015 = []
    P3_Kalman_000015 = []
    
    for i in range(Samples):
        
        S1_Raw = Get_Reading("S1")
        S2_Raw = Get_Reading("S2")
        S3_Raw = Get_Reading("S3")
        RAW_S1.append(S1_Raw)
        RAW_S2.append(S2_Raw)
        RAW_S3.append(S3_Raw)
        
        S1_MVA, S2_MVA, S3_MVA = Moving_Average_Filter_Updated(S1_Raw, S2_Raw, S3_Raw)
        S1_MVA_3, S2_MVA_3, S3_MVA_3 = Moving_Average_Filter_Updated_3(S1_Raw, S2_Raw, S3_Raw)
        S1_MVA_8, S2_MVA_8, S3_MVA_8 = Moving_Average_Filter_Updated_8(S1_Raw, S2_Raw, S3_Raw)
        S1_MVA_10, S2_MVA_10, S3_MVA_10 = Moving_Average_Filter_Updated_10(S1_Raw, S2_Raw, S3_Raw)
        
#         if i == 0:
#             Kalman_Raw = Kalman(True, S1_MVA, S2_MVA, S3_MVA,0.015)
#             Kalman_Raw_015 = Kalman(True, S1_MVA, S2_MVA, S3_MVA,0.15)
#             Kalman_Raw_00015 = Kalman(True, S1_MVA, S2_MVA, S3_MVA,0.0015)
#             Kalman_Raw_000015 = Kalman(True, S1_MVA, S2_MVA, S3_MVA,0.00015)
#         else:
#             Kalman_Raw = Kalman(False, S1_MVA, S2_MVA, S3_MVA,0.015)
#             Kalman_Raw_015 = Kalman(False, S1_MVA, S2_MVA, S3_MVA,0.15)
#             Kalman_Raw_00015 = Kalman(False, S1_MVA, S2_MVA, S3_MVA,0.0015)
#             Kalman_Raw_000015 = Kalman(False, S1_MVA, S2_MVA, S3_MVA,0.00015)
#             
#         S1_Kal = int(Kalman_Raw[0])
#         S2_Kal = int(Kalman_Raw[1])
#         S3_Kal = int(Kalman_Raw[2])
#         S1_Kal_015 = int(Kalman_Raw_015[0])
#         S2_Kal_015 = int(Kalman_Raw_015[1])
#         S3_Kal_015 = int(Kalman_Raw_015[2])
#         S1_Kal_00015 = int(Kalman_Raw_00015[0])
#         S2_Kal_00015 = int(Kalman_Raw_00015[1])
#         S3_Kal_00015 = int(Kalman_Raw_00015[2])
#         S1_Kal_000015 = int(Kalman_Raw_000015[0])
#         S2_Kal_000015 = int(Kalman_Raw_000015[1])
#         S3_Kal_000015 = int(Kalman_Raw_000015[2])
        
#         S1_MVA = int(S1_MVA)
#         S2_MVA = int(S2_MVA)
#         S3_MVA = int(S3_MVA)
#         S1_MVA_3 = int(S1_MVA_3)
#         S2_MVA_3 = int(S2_MVA_3)
#         S3_MVA_3 = int(S3_MVA_3)
#         S1_MVA_8 = int(S1_MVA_8)
#         S2_MVA_8 = int(S2_MVA_8)
#         S3_MVA_8 = int(S3_MVA_8)
#         S1_MVA_10 = int(S1_MVA_10)
#         S2_MVA_10 = int(S2_MVA_10)
#         S3_MVA_10 = int(S3_MVA_10)
        S1_MVA = S1_MVA
        S2_MVA = S2_MVA
        S3_MVA = S3_MVA
        S1_MVA_3 = S1_MVA_3
        S2_MVA_3 = S2_MVA_3
        S3_MVA_3 = S3_MVA_3
        S1_MVA_8 = S1_MVA_8
        S2_MVA_8 = S2_MVA_8
        S3_MVA_8 = S3_MVA_8
        S1_MVA_10 = S1_MVA_10
        S2_MVA_10 = S2_MVA_10
        S3_MVA_10 = S3_MVA_10
        
        
        S1_V_Raw = Get_Voltage(S1_Raw)
        S2_V_Raw = Get_Voltage(S2_Raw)
        S3_V_Raw = Get_Voltage(S3_Raw)
        
#         S1_Kal_V = Get_Voltage(S1_Kal)
#         S2_Kal_V = Get_Voltage(S2_Kal)
#         S3_Kal_V = Get_Voltage(S3_Kal)
#         
#         S1_Kal_V_015 = Get_Voltage(S1_Kal_015)
#         S2_Kal_V_015 = Get_Voltage(S2_Kal_015)
#         S3_Kal_V_015 = Get_Voltage(S3_Kal_015)
#         
#         S1_Kal_V_00015 = Get_Voltage(S1_Kal_00015)
#         S2_Kal_V_00015 = Get_Voltage(S2_Kal_00015)
#         S3_Kal_V_00015 = Get_Voltage(S3_Kal_00015)
#         
#         S1_Kal_V_000015 = Get_Voltage(S1_Kal_000015)
#         S2_Kal_V_000015 = Get_Voltage(S2_Kal_000015)
#         S3_Kal_V_000015 = Get_Voltage(S3_Kal_000015)
        
        S1_MVA_V = Get_Voltage(S1_MVA)
        S2_MVA_V = Get_Voltage(S2_MVA)
        S3_MVA_V = Get_Voltage(S3_MVA)
        
        S1_MVA_V_3 = Get_Voltage(S1_MVA_3)
        S2_MVA_V_3 = Get_Voltage(S2_MVA_3)
        S3_MVA_V_3 = Get_Voltage(S3_MVA_3)
        
        S1_MVA_V_8 = Get_Voltage(S1_MVA_8)
        S2_MVA_V_8 = Get_Voltage(S2_MVA_8)
        S3_MVA_V_8 = Get_Voltage(S3_MVA_8)
        
        S1_MVA_V_10 = Get_Voltage(S1_MVA_10)
        S2_MVA_V_10 = Get_Voltage(S2_MVA_10)
        S3_MVA_V_10 = Get_Voltage(S3_MVA_10)
        
        S1_MVA_P = (S1_MVA_V-Sensor_Offsets_Gen[0])*Scaler
        S2_MVA_P = (S2_MVA_V-Sensor_Offsets_Gen[1])*Scaler
        S3_MVA_P = (S3_MVA_V-Sensor_Offsets_Gen[2])*Scaler
        
        S1_MVA_P_3 = (S1_MVA_V_3-Sensor_Offsets_Gen[0])*Scaler
        S2_MVA_P_3 = (S2_MVA_V_3-Sensor_Offsets_Gen[1])*Scaler
        S3_MVA_P_3 = (S3_MVA_V_3-Sensor_Offsets_Gen[2])*Scaler
        
        S1_MVA_P_8 = (S1_MVA_V_8-Sensor_Offsets_Gen[0])*Scaler
        S2_MVA_P_8 = (S2_MVA_V_8-Sensor_Offsets_Gen[1])*Scaler
        S3_MVA_P_8 = (S3_MVA_V_8-Sensor_Offsets_Gen[2])*Scaler
        
        S1_MVA_P_10 = (S1_MVA_V_10-Sensor_Offsets_Gen[0])*Scaler
        S2_MVA_P_10 = (S2_MVA_V_10-Sensor_Offsets_Gen[1])*Scaler
        S3_MVA_P_10 = (S3_MVA_V_10-Sensor_Offsets_Gen[2])*Scaler
        
#         S1_Kal_P = (S1_Kal_V-Sensor_Offsets_Gen[0])*Scaler
#         S2_Kal_P = (S2_Kal_V-Sensor_Offsets_Gen[1])*Scaler
#         S3_Kal_P = (S3_Kal_V-Sensor_Offsets_Gen[2])*Scaler
#         
#         S1_Kal_P_015 = (S1_Kal_V_015-Sensor_Offsets_Gen[0])*Scaler
#         S2_Kal_P_015 = (S2_Kal_V_015-Sensor_Offsets_Gen[1])*Scaler
#         S3_Kal_P_015 = (S3_Kal_V_015-Sensor_Offsets_Gen[2])*Scaler
#         
#         S1_Kal_P_00015 = (S1_Kal_V_00015-Sensor_Offsets_Gen[0])*Scaler
#         S2_Kal_P_00015 = (S2_Kal_V_00015-Sensor_Offsets_Gen[1])*Scaler
#         S3_Kal_P_00015 = (S3_Kal_V_00015-Sensor_Offsets_Gen[2])*Scaler
#         
#         S1_Kal_P_000015 = (S1_Kal_V_000015-Sensor_Offsets_Gen[0])*Scaler
#         S2_Kal_P_000015 = (S2_Kal_V_000015-Sensor_Offsets_Gen[1])*Scaler
#         S3_Kal_P_000015 = (S3_Kal_V_000015-Sensor_Offsets_Gen[2])*Scaler
#         
        V1_Raw.append(S1_V_Raw)
        V2_Raw.append(S2_V_Raw)
        V3_Raw.append(S3_V_Raw)
        
        V1_MVA.append(S1_MVA_V)
        V2_MVA.append(S2_MVA_V)
        V3_MVA.append(S3_MVA_V)
        V1_MVA_3.append(S1_MVA_V_3)
        V2_MVA_3.append(S2_MVA_V_3)
        V3_MVA_3.append(S3_MVA_V_3)
        V1_MVA_8.append(S1_MVA_V_8)
        V2_MVA_8.append(S2_MVA_V_8)
        V3_MVA_8.append(S3_MVA_V_8)
        V1_MVA_10.append(S1_MVA_V_10)
        V2_MVA_10.append(S2_MVA_V_10)
        V3_MVA_10.append(S3_MVA_V_10)
        
#         V1_Kalman.append(S1_Kal_V)
#         V2_Kalman.append(S2_Kal_V)
#         V3_Kalman.append(S3_Kal_V)
#         V1_Kalman_015.append(S1_Kal_V_015)
#         V2_Kalman_015.append(S2_Kal_V_015)
#         V3_Kalman_015.append(S3_Kal_V_015)
#         V1_Kalman_00015.append(S1_Kal_V_00015)
#         V2_Kalman_00015.append(S2_Kal_V_00015)
#         V3_Kalman_00015.append(S3_Kal_V_00015)
#         V1_Kalman_000015.append(S1_Kal_V_000015)
#         V2_Kalman_000015.append(S2_Kal_V_000015)
#         V3_Kalman_000015.append(S3_Kal_V_000015)
         
        P1_MVA.append(S1_MVA_P)
        P2_MVA.append(S2_MVA_P)
        P3_MVA.append(S3_MVA_P)
        P1_MVA_3.append(S1_MVA_P_3)
        P2_MVA_3.append(S2_MVA_P_3)
        P3_MVA_3.append(S3_MVA_P_3)
        P1_MVA_8.append(S1_MVA_P_8)
        P2_MVA_8.append(S2_MVA_P_8)
        P3_MVA_8.append(S3_MVA_P_8)
        P1_MVA_10.append(S1_MVA_P_10)
        P2_MVA_10.append(S2_MVA_P_10)
        P3_MVA_10.append(S3_MVA_P_10)
        
#         P1_Kalman.append(S1_Kal_P)
#         P2_Kalman.append(S2_Kal_P)
#         P3_Kalman.append(S3_Kal_P)
#         P1_Kalman_015.append(S1_Kal_P_015)
#         P2_Kalman_015.append(S2_Kal_P_015)
#         P3_Kalman_015.append(S3_Kal_P_015)
#         P1_Kalman_00015.append(S1_Kal_P_00015)
#         P2_Kalman_00015.append(S2_Kal_P_00015)
#         P3_Kalman_00015.append(S3_Kal_P_00015)
#         P1_Kalman_000015.append(S1_Kal_P_000015)
#         P2_Kalman_000015.append(S2_Kal_P_000015)
#         P3_Kalman_000015.append(S3_Kal_P_000015)
        
        print(i)
    
    print(min(V1_Raw))
    print(min(V2_Raw))
    print(min(V3_Raw))
    
    print(min(V1_MVA))
    print(min(V2_MVA))
    print(min(V3_MVA))
    
    Test_File = open(r"Compare","a")
    
    Test_File.write("\nMVA 5 P: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(P1_MVA))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(P3_MVA))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(P3_MVA))
    Test_File.flush()

    Test_File.write("\nMVA 3 P: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(P1_MVA_3))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(P3_MVA_3))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(P3_MVA_3))
    Test_File.flush()
    
    Test_File.write("\nMVA 8 P: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(P1_MVA_8))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(P3_MVA_8))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(P3_MVA_8))
    Test_File.flush()
    
    Test_File.write("\nMVA 10 P: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(P1_MVA_10))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(P3_MVA_10))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(P3_MVA_10))
    Test_File.flush()
    
    Test_File.write("\nMVA 5 V: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(V1_MVA))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(V3_MVA))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(V3_MVA))
    Test_File.flush()
    
    Test_File.write("\nMVA 3 V: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(V1_MVA_3))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(V3_MVA_3))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(V3_MVA_3))
    Test_File.flush()
    
    Test_File.write("\nMVA 8 V: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(V1_MVA_8))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(V3_MVA_8))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(V3_MVA_8))
    Test_File.flush()
    
    Test_File.write("\nMVA 10 V: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(V1_MVA_10))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(V3_MVA_10))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(V3_MVA_10))
    Test_File.flush()
    
    Test_File.write("\nRaw: ")
    Test_File.flush()
    Test_File.write("\nS1 Data: " + str(RAW_S1))
    Test_File.flush()
    Test_File.write("\nS2 Data: " + str(RAW_S1))
    Test_File.flush()
    Test_File.write("\nS3 Data: " + str(RAW_S1))
    Test_File.flush()
    

#     Test_File.write("\nKalman 0.015 P: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(P1_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(P3_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(P3_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.15 P: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(P1_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(P3_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(P3_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.0015 P: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(P1_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(P3_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(P3_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.00015 P: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(P1_Kalman_000015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(P3_Kalman_000015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(P3_Kalman_000015))
#     time.sleep(Delay_Short)
#     
#     Test_File.write("\nKalman 0.015 V: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(V1_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(V3_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(V3_Kalman))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.15 V: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(V1_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(V3_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(V3_Kalman_015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.0015 V: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(V1_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(V3_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(V3_Kalman_00015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nKalman 0.00015 V: ")
#     time.sleep(Delay_Short)
#     Test_File.write("\nS1 Data: " + str(V1_Kalman_000015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS2 Data: " + str(V3_Kalman_000015))
#     time.sleep(Delay_Short)
#     Test_File.write("\nS3 Data: " + str(V3_Kalman_000015))
#     time.sleep(Delay_Short)
    
    Test_File.close()
 

#---------------------------------- FIR STUFF --------------------------------------


# Hamming bandpass FIR filter
def Bandpass_FIR(N_Samples, Low_Cut, High_Cut, Sample_Rate, Window='hamming'):
    # Find nyquist rate
    Nyquist = 0.5*Sample_Rate
    
    # Calc Filter
    Filter = firwin(N_Samples, [Low_Cut,High_Cut], nyq=Nyquist, pass_zero=False, window=Window, scale=False)
    
    return Filter

def Gen_Sine(Frequency,Sample_Rate,Duration):
    x = np.linspace(0,Duration,Sample_Rate*Duration,endpoint=False)
    Freq = x*Frequency
    y = np.sin((2*np.pi)*Freq)
    
    return x,y

def Test_Test():
    sample_rate = 100.0
    nsamples = 400
    t = np.arange(nsamples) / sample_rate
    x = np.cos(2*np.pi*0.5*t) + 0.2*np.sin(2*np.pi*2.5*t+0.1) + 0.2*np.sin(2*np.pi*15.3*t) + 0.1*np.sin(2*np.pi*16.7*t + 0.1) + 0.1*np.sin(2*np.pi*23.45*t+0.8)
    
    Low_Cut = 0.1
    High_Cut = 5
    N_Samples = 400
    
    Hamming_Fir = Bandpass_FIR(N_Samples,Low_Cut,High_Cut,Sample_Rate=sample_rate)
    Filtered_x = lfilter(Hamming_Fir,1.0,x)
    plt.figure(figsize=(6,5))
    plt.plot(t,Filtered_x,label='Filtered Signal')
    plt.show()

def Test_Air_Fir():
    # Show data
    Data = [16194, 16190, 16192, 16193, 16196, 16202, 16204, 16206, 16209, 16209, 16211, 16213, 16212, 16215, 16215, 16209, 16208, 16210, 16209, 16209, 16208, 16209, 16213, 16212, 16212, 16217, 16216, 16213, 16213, 16213, 16216, 16217, 16218, 16218, 16220, 16220, 16222, 16224, 16224, 16224, 16225, 16223, 16221, 16220, 16220, 16220, 16219, 16218, 16216, 16216, 16212, 16210, 16208, 16207, 16204, 16201, 16200, 16202, 16200, 16202, 16203, 16207, 16208, 16211, 16214, 16219, 16220, 16222, 16222, 16222, 16224, 16224, 16225, 16225, 16225, 16228, 16227, 16226, 16229, 16227, 16226, 16226, 16227, 16226, 16226, 16222, 16221, 16223, 16222, 16224, 16224, 16228, 16225, 16224, 16226, 16224, 16224, 16221, 16223, 16222, 16221, 16218, 16220, 16221, 16222, 16222, 16224, 16225, 16225, 16224, 16224, 16223, 16222, 16221, 16221, 16222, 16225, 16224, 16221, 16219, 16218, 16217, 16216, 16214, 16211, 16210, 16207, 16207, 16208, 16210, 16212, 16213, 16217, 16219, 16220, 16220, 16220, 16219, 16223, 16224, 16223, 16224, 16222, 16220, 16222, 16221, 16222, 16223, 16222, 16221, 16219, 16221, 16221, 16222, 16221, 16220, 16221, 16222, 16222, 16221, 16224, 16220, 16221, 16220, 16223, 16223, 16224, 16220, 16220, 16218, 16218, 16217, 16217, 16216, 16213, 16211, 16209, 16209, 16209, 16209, 16207, 16207, 16208, 16207, 16207, 16204, 16205, 16207, 16206, 16206, 16204, 16206, 16204, 16205, 16205, 16208, 16209, 16211, 16211, 16211, 16213, 16213, 16212, 16213, 16211, 16210, 16209, 16205, 16205, 16206, 16207, 16207, 16207, 16208, 16208, 16209, 16209, 16210, 16211, 16211, 16209, 16210, 16211, 16212, 16211, 16200, 16199, 16199, 16197, 16197, 16196, 16196, 16197, 16197, 16200, 16213, 16213, 16215, 16218, 16220, 16221, 16220, 16220, 16218, 16218, 16216, 16218, 16217, 16219, 16217, 16215, 16214, 16212, 16213, 16214, 16215, 16214, 16217, 16217, 16219, 16220, 16222, 16224, 16222, 16220, 16220, 16220, 16219, 16215, 16215, 16216, 16215, 16210, 16214, 16210, 16209, 16207, 16207, 16208, 16212, 16210, 16212, 16217, 16216, 16218, 16219, 16220, 16219, 16220, 16218, 16221, 16224, 16221, 16223, 16226, 16228, 16228, 16232, 16231, 16231, 16226, 16224, 16225, 16223, 16221, 16223, 16224, 16219, 16220, 16217, 16218, 16217, 16218, 16221, 16222, 16216, 16212, 16212, 16211, 16212, 16211, 16211, 16210, 16207, 16207, 16209, 16214, 16216, 16218, 16219, 16220, 16219, 16220, 16222, 16223, 16221, 16221, 16220, 16216, 16216, 16219, 16219, 16219, 16218, 16219, 16222, 16221, 16221, 16224, 16225, 16223, 16223, 16224, 16223, 16225, 16224, 16224, 16224, 16225, 16226, 16227, 16225, 16224, 16225, 16220, 16220, 16219, 16217, 16215, 16211, 16211, 16210, 16212, 16209, 16213, 16213, 16215, 16216, 16218, 16218, 16221, 16220, 16218, 16217, 16214, 16211, 16212, 16211, 16213, 16214, 16217, 16218, 16221, 16220, 16222, 16221, 16219, 16219, 16218, 16217, 16213, 16212, 16211, 16212, 16212, 16214, 16217, 16217, 16217, 16218, 16219, 16222, 16225, 16224, 16225, 16222, 16219, 16221, 16220, 16220, 16218, 16220, 16218, 16218, 16218, 16222, 16224, 16222, 16221, 16222, 16222, 16220, 16219, 16221, 16219, 16216, 16216, 16217, 16215, 16216, 16216, 16216, 16215, 16215, 16216, 16218, 16218, 16217, 16217, 16215, 16212, 16212, 16211, 16210, 16209, 16210, 16209, 16210, 16212, 16216, 16218, 16217, 16219, 16217, 16219, 16222, 16221, 16221, 16221, 16219, 16217, 16218, 16216, 16220, 16218, 16215, 16213, 16212, 16212, 16214, 16215, 16215, 16217, 16217, 16218, 16215, 16215, 16216, 16214, 16213, 16212, 16214, 16211, 16210, 16206, 16211, 16213, 16215, 16214, 16215]
    Data = np.array(Data)
    #Data = detrend(Data)
    Time_Steps = np.arange(0,500,1)
    plt.figure(figsize=(6,5))
    plt.plot(Time_Steps,Data,label='Signal')
    plt.show()
    
    # fft
    Data_FFT = fft(Data)
    Power = np.abs(Data_FFT)**2
    Sample_Freq = fftfreq(Data.size,d=1/0.333)
    
#     plt.figure(figsize=(6,5))
#     plt.plot(Sample_Freq, Power)
#     plt.xlabel('Freq [Hz]')
#     plt.ylabel('Power')
    
    # Find peak freq
    #Mask = np.where(Sample_Freq > 0)
    #Freqs = Sample_Freq[Mask]
    
    #Peak_Freq = Freqs[Power[Mask].argmax()]
    #np.allclose(Peak_Freq, 1./500)
    #Axes = plt.axes([0.55,0.3,0.3,0.5])
#     plt.title('Peak frequency')
#     plt.plot(Freqs[:8],Power[Mask][:8])
#     plt.setp(Axes,yticks=[])
#     plt.show()
    
    # Filter
    Filtered_FFT = Data_FFT.copy()
    
    # Make filter
    # Test values
    Sample_Rate = 3.333
    Low_Cut = 0.003
    High_Cut = 0.007
    N_Samples = 500
    Nyquist = 0.5*Sample_Rate
    
    Hamming_Fir = Bandpass_FIR(N_Samples,Low_Cut,High_Cut,Sample_Rate=Sample_Rate)
    
    w,h = freqz(Hamming_Fir,worN=2000)
    
#     plt.plot((w/np.pi)*Nyquist,abs(h),linewidth=2,label="Hamming Window")
#     plt.xlabel('Frequency (Hz)')
#     plt.ylabel('Gain')
#     plt.title('Frequency response')
#     plt.xlim(0,0.1)
#     plt.ylim(0,1.1)
#     plt.grid(True)
#     plt.legend()
#     plt.xlabel('Frequency (Hz)')
#     plt.ylabel('Gain')
#     plt.show()
    
    Filtered_x = lfilter(Hamming_Fir,1.0,Data)
    plt.figure(figsize=(6,5))
    plt.plot(Time_Steps,Filtered_x,label='Filtered Signal')
    plt.show()
    
    
    

def Ano_Test():
    
    # Make signal
    np.random.seed(1234)
    time_step = 0.02
    period = 5.0
    time_vec = np.arange(0,20,time_step)
    sig = (np.sin(2*np.pi / period*time_vec)+ 0.5 * np.random.randn(time_vec.size))
    plt.figure(figsize=(6,5))
    plt.plot(time_vec,sig,label='Fuck')
    #plt.show()
    
    # Make fft
    sig_fft = fft(sig)
    # get power
    power = np.abs(sig_fft)**2
    # Get frequencies
    sample_freq = fftfreq(sig.size,d=time_step)
    # plot
    #plt.figure(figsize=(6,5))
    #plt.plot(sample_freq, power)
    #plt.xlabel('Freq [Hz]')
    #plt.ylabel('Power')
    # Find peak freq
    pos_mask = np.where(sample_freq > 0)
    freqs = sample_freq[pos_mask]
    peak_freq = freqs[power[pos_mask].argmax()]
    np.allclose(peak_freq, 1./period)
    axes = plt.axes([0.55,0.3,0.3,0.5])
    #plt.title('Peak frequency')
    #plt.plot(freqs[:8],power[pos_mask][:8])
    #plt.setp(axes,yticks=[])
    #plt.show()
    # Look into scipy.signal.find_peaks_cwt
    
    # Remove high freq
    high_freq_fft = sig_fft.copy()
    
    # Make filter
    # Test values
    Sample_Rate = 63.0
    Low_Cut = 0.7
    High_Cut = 4.0
    N_Samples = 128
    
    Hamming_Fir = Bandpass_FIR(N_Samples,Low_Cut,High_Cut,Sample_Rate=Sample_Rate)
    
    w,h = freqz(Hamming_Fir,1,worN=2000)
    
    
    
def FFT_Test():
    # Generate test sine wave
    Sample_Rate = 44100 # Hz
    Duration = 5 # s
    N = Sample_Rate*Duration
    
    x,y = Gen_Sine(2,Sample_Rate,Duration)
    
    yf = fft(y)
    xf = np
    #xf = fft(N,1/Sample_Rate)
    
    #_, Nice = Gen_Sine(400,Sample_Rate,Duration)
    #_, Noice = Gen_Sine(4000,Sample_Rate,Duration)
    #Noice_Tone = Noice*0.3
    #Final = Nice+Noice_Tone
    
    # Normalize
    #Norm_Tone = np.int16(Final/Final.max())*32767
    
    # plot
    plt.plot(x,y)
    plt.show()
    
    #plt.plot(xf,np.abs(yf))
    #plt.show()

def Filter_Test():
    # Test values
    Sample_Rate = 63.0
    Low_Cut = 0.7
    High_Cut = 4.0
    N_Samples = 128
    
    Hamming_Fir = Bandpass_FIR(N_Samples,Low_Cut,High_Cut,Sample_Rate=Sample_Rate)
    
    w,h = freqz(Hamming_Fir,1,worN=2000)
    plt.plot((Sample_Rate*0.5/np.pi)*w,abs(h),label="Hamming Window")
    plt.xlim(0,8.0)
    plt.ylim(0,1.1)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain')
    plt.show()

def Sampler():
    
    # Go through every sample for the test
    for i in range(Samples):
        # Get new readings
        S1_Raw = Get_Reading("S1")
        S2_Raw = Get_Reading("S2")
        S3_Raw = Get_Reading("S3")

    


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

#Sample_Test()
#Filter_Test()
#FFT_Test()
Test_Air_Fir()
#Test_Test()
#Signal_Processing()
#Comparison_Test()

GPIO.output(LED, GPIO.LOW)

#Move_Stop()
