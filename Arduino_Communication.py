# Imports
import serial

# Variable initialisation
Voltage_Readings = {"V1":{}, "V2":{},"V3":{}}
Newest_Reading = {"V1":0, "V2":0,"V3":0}
Voltage_Offsets = {"V1":0.46875, "V2":0,"V3":0}

Pressure_Readings = {"P1":{}, "P2":{},"P3":{}}
Current_Pressure = {"P1":0, "P2":0,"P3":0}

# Functions
def Read_Float(String):
    # Define location variables
    Float_Start = -1
    Float_End = -1
    # Go through each letter
    for Letter in range(len(String)):
        # If first reached digit
        if Float_Start == -1 and String[Letter].isdigit() == True and Letter > 2:
            Float_Start = Letter
        # If Last digit has been reached
        elif Float_End == -1 and String[Letter].isdigit() == False and String[Letter] != '.' and Float_Start != -1:
            Float_End = Letter
    
    Float_Str = String[Float_Start:Float_End]
    return float(Float_Str)

def Get_Voltage(Message):
    # Get float value and convert to read voltage
    V_Float = Read_Float(Message)*5/1024
    # Update current voltage value
    Current_V = Message[0:2]
    Newest_Reading[Current_V] = V_Float
    # Check if values found before
    if V_Float in Voltage_Readings[Current_V]:
        Voltage_Readings[Current_V][V_Float] += 1
    else:
        Voltage_Readings[Current_V][V_Float] = 1
    return Current_V
        
def Get_Pressure(Current_Sensor):
    Current_Sensor_P = 'P'+Current_Sensor[1]
    # Update current pressure
    Pressure = (Newest_Reading[Current_Sensor]-Voltage_Offsets[Current_Sensor])*250
    Current_Pressure[Current_Sensor_P] = Pressure
    
    # Update pressure readings
    if Pressure in Pressure_Readings[Current_Sensor_P]:
        Pressure_Readings[Current_Sensor_P][Pressure] += 1
    else:
        Pressure_Readings[Current_Sensor_P][Pressure] = 1
        
def Determine_Height():
    print("Oh no")

# Initialise my serial communication
if __name__ == '__main__':
    Ser_Ard = serial.Serial('/dev/ttyACM0',115200,timeout=1)
    
    # Reset our serial buffer
    Ser_Ard.reset_input_buffer()
    
    # My main loop
    while True:
        # If there is a message to reads
        if Ser_Ard.in_waiting > 0:
            # Read message
            Message = Ser_Ard.readline().decode('utf-8').rstrip()
            # Printing entire message
            print(Message)
            if len(Message) > 3 and Message[0] == 'V':
                # Calculate voltage and update global voltage variables
                Current_Sensor = Get_Voltage(Message)
                # Calculate sensor pressure
                Get_Pressure(Current_Sensor)
            
            # Test
            print("Newest V:")
            print(Newest_Reading)
            print("All V:")
            print(Voltage_Readings)
            print("Newest P:")
            print(Current_Pressure)
            print("All P:")
            print(Pressure_Readings)


