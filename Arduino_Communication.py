# Imports
import serial
import time

# Variable initialisation
Voltage_Readings = {"V1":{}, "V2":{},"V3":{}}
Newest_Reading = {"V1":0, "V2":0,"V3":0}
Voltage_Offsets = {"V1":0.47695312500000003, "V2":0.47695312500000003,"V3":0.47695312500000003}

Pressure_Readings = {"P1":{}, "P2":{},"P3":{}}
Current_Pressure = {"P1":0, "P2":0,"P3":0}

Reference_Voltage = 1.1
Resolution = 1024
Scaler = 250
Serial_Bandwidth = 115200
Padding_Long = 25
Padding_Short = 6
Sampling = 5000



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
    V_Float = Read_Float(Message)*Reference_Voltage/Resolution
    # Update current voltage value
    Current_V = 'V'+Message[1]
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
    Pressure = (Newest_Reading[Current_Sensor]-Voltage_Offsets[Current_Sensor])*Scaler
    Current_Pressure[Current_Sensor_P] = Pressure
    
    # Update pressure readings
    if Pressure in Pressure_Readings[Current_Sensor_P]:
        Pressure_Readings[Current_Sensor_P][Pressure] += 1
    else:
        Pressure_Readings[Current_Sensor_P][Pressure] = 1

def Prepare_Data():
    # Initialise lists
    Voltage_List = []
    Pressure_List = []
    
    # Make list of every read value
    for Sensor in Voltage_Readings:
        for Voltage in Voltage_Readings[Sensor]:
            Voltage_List.append(Voltage)
        for Pressure in Pressure_Readings['P'+Sensor[1]]:
            Pressure_List.append(Pressure)
    # Remove duplicates
    Voltage_List = list(set(Voltage_List))
    Pressure_List = list(set(Pressure_List))
    
    # Sort the lists
    Voltage_List.sort()
    Pressure_List.sort()
    
    return Voltage_List,Pressure_List

def Main_Samples(Samples):
    
    # Initialise my serial communication
    if __name__ == '__main__':
        Ser_Ard = serial.Serial('/dev/ttyACM0',Serial_Bandwidth,timeout=1)
    
        # Reset our serial buffer
        Ser_Ard.reset_input_buffer()
    
        # Initialise Samples left
        Samples_Left = Samples
    
        # Initialise message checker
        Received_Sensors = {"S1":0, "S2":0,"S3":0}
    
        # Countdown loop
        while Samples_Left > 0:
        
            # If there is a message to reads
            if Ser_Ard.in_waiting > 0:
                # Read message
                Message = Ser_Ard.readline().decode('utf-8').rstrip()
                # Printing entire message
                print(Message)
                if len(Message) > 12 and Message[0] == 'S':
                    # Calculate voltage and update global voltage variables
                    Current_Sensor = Get_Voltage(Message)
                    # Calculate sensor pressure
                    Get_Pressure(Current_Sensor)
                    # Update received sensor info
                    Received_Sensors[Message[0:2]] = Received_Sensors[Message[0:2]]+1
                
            
            # Check if each sensor have given info
            if Received_Sensors["S1"] == 1 and Received_Sensors["S2"] == 1 and Received_Sensors["S3"] == 1:
                # Reset values
                Received_Sensors["S1"] = 0
                Received_Sensors["S2"] = 0
                Received_Sensors["S3"] = 0
                # Print info
                #print("Newest V:")
                #print(Newest_Reading)
                #print("All V:")
                #print(Voltage_Readings)
                #print("Newest P:")
                #print(Current_Pressure)
                #print("All P:")
                #print(Pressure_Readings)
                #print("Current Sample:")
                print(Samples_Left)
            
                # Count down samples left
                Samples_Left -= 1
    
        # Preparing data for Being sent
        Voltage_List,Pressure_List = Prepare_Data()
    
        # Write results to file
        Test_File = open("Sensor_Test","a")
    
        # Write start line
        Test_File.write("\nVoltage:                 Pressure:                S1:   S2:   S3:\n")
        # Prepare and append strings
        for i in range(len(Voltage_List)):
            # Initialise sub strings
            V_String = str(Voltage_List[i])
            V_String = V_String.ljust(Padding_Long,' ')
        
            P_String = str(Pressure_List[i])
            P_String = P_String.ljust(Padding_Long,' ')
        
            if Voltage_List[i] in Voltage_Readings["V1"]:
                S1_String = str(Voltage_Readings["V1"][Voltage_List[i]])
                S1_String = S1_String.ljust(Padding_Short,' ')
            else:
                S1_String = str("0")
                S1_String = S1_String.ljust(Padding_Short,' ')
                
            if Voltage_List[i] in Voltage_Readings["V2"]:
                S2_String = str(Voltage_Readings["V2"][Voltage_List[i]])
                S2_String = S2_String.ljust(Padding_Short,' ')
            else:
                S2_String = str("0")
                S2_String = S2_String.ljust(Padding_Short,' ')
        
            if Voltage_List[i] in Voltage_Readings["V3"]:
                S3_String = str(Voltage_Readings["V3"][Voltage_List[i]])
                S3_String = S3_String.ljust(Padding_Short,' ')
            else:
                S3_String = str("0")
                S3_String = S3_String.ljust(Padding_Short,' ')
            S3_String = S3_String+"\n"
        
            # Complete string
            Final_String = V_String+P_String+S1_String+S2_String+S3_String
        
            # Write string
            Test_File.write(Final_String)
            
        # Close file
        Test_File.close()
    
# Main
Main_Samples(Sampling)

# Initialise my serial communication
#if __name__ == '__main__':
    #Ser_Ard = serial.Serial('/dev/ttyACM0',Serial_Bandwidth,timeout=1)
    
    # Reset our serial buffer
    #Ser_Ard.reset_input_buffer()
    
    # My main loop
    #while True:
        # If there is a message to reads
        #if Ser_Ard.in_waiting > 0:
            # Read message
            #Message = Ser_Ard.readline().decode('utf-8').rstrip()
            # Printing entire message
            #print(Message)
            #if len(Message) > 3 and Message[0] == 'S':
                # Calculate voltage and update global voltage variables
                #Current_Sensor = Get_Voltage(Message)
                # Calculate sensor pressure
                #Get_Pressure(Current_Sensor)
            
            # Test
            #print("Newest V:")
            #print(Newest_Reading)
            #print("All V:")
            #print(Voltage_Readings)
            #print("Newest P:")
            #print(Current_Pressure)
            #print("All P:")
            #print(Pressure_Readings)
            
            #Total_Readings_S1 = sum(Voltage_Readings['V1'].values())
            #Total_Readings_S2 = sum(Voltage_Readings['V2'].values())
            #Total_Readings_S3 = sum(Voltage_Readings['V3'].values())
            #print("Total readings S1: ")
            #print(Total_Readings_S1)
            #print("Total readings S2: ")
            #print(Total_Readings_S2)
            #print("Total readings S3: ")
            #print(Total_Readings_S3)
                


