# Imports
import serial

# Variable initialisation
Pressure_Readings = []

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
            #Message += Ser_Ard.readline()
            # If message is a string we will print it
            if isinstance(Message,str) == True:
                print(Message)
            elif isinstance(Message, float) == True:
                Pressure_Readings.append(Message)
            else:
                print(type(Message))
                #print("Error: Serial message type unknown")