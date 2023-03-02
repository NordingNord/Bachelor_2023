// Pressure sensor offsets
//const float offset_1 = 0.474, offset_2 = 0.474, offset_3 = 0.474, offset_4 = 0.474, offset_5 = 0.474;
// Pressure sensor pins
const int Sensor_Pin_1 = 0, Sensor_Pin_2 = 1, Sensor_Pin_3 = 2, Sensor_Pin_4 = 3, Sensor_Pin_5 = 4;

// Variables for voltage and pressure
float V1, V2, V3, V4;
//float P1, P2, P3, P4;

float Increase_Bits(int Bits){
  // Initialise value sums
  uint32_t V1_Sum =0;
  uint32_t V2_Sum =0;
  uint32_t V3_Sum =0 ;

  // Take corresponding samples
  for(int i = 0; i < pow(4,Bits); i++){
    V1_Sum += analogRead(Sensor_Pin_1);
    V2_Sum += analogRead(Sensor_Pin_2);
    V3_Sum += analogRead(Sensor_Pin_3);
  }

  // Average the sums
  V1_Sum = V1_Sum/pow(4,Bits);
  V2_Sum = V2_Sum/pow(4,Bits);
  V3_Sum = V3_Sum/pow(4,Bits);

  // Decimate the position by bit shifting right
  uint32_t V1_Result = (V1_Sum >> Bits);
  uint32_t V2_Result = (V2_Sum >> Bits);
  uint32_t V3_Result = (V3_Sum >> Bits);

  return float(V1_Result), float(V2_Result), float(V3_Result);
}

void setup() {
  // Setup serial communication with a baud rate of 115200 bps
  Serial.begin(115200);
  //Serial.println("Sensor readings have begun.");
}

void loop() {
  //V1,V2,V3 = Increase_Bits(2);
  // Calculate output voltage. Go from analog to digitial by timing it with the maximum voltage 5V devided by 1024, since we have a 10 bit converter
  V1 = analogRead(Sensor_Pin_1);// * 5.00/1024;
  //V2 = analogRead(Sensor_Pin_2);// * 5.00/1024;
  //V3 = analogRead(Sensor_Pin_3);// * 5.00/1024;
  //V4 = analogRead(Sensor_Pin_4) //* 5.00/1024;
  // Calculate water pressure
  //P1 = (V1-offset_1) * 250;
  //P2 = (V2-offset_2) * 250;
  //P3 = (V3-offset_3) * 250;
  //P4 = (1-offset_4) * 250;

  // Print pressure and voltages
  //Serial.println("Voltage 1 (V): ");
  //Serial.println(V1, 3);
  //Serial.println("Pressure 1 (Kpa): ");
  //Serial.println(P1, 3);

  //Serial.println("Voltage 2 (V): ");
  //Serial.println(V2, 3);
  //Serial.println("Pressure 2 (Kpa): ");
  //Serial.println(P2, 3);

  //Serial.println("Voltage 3 (V): ");
  //Serial.println(V3, 3);
  //Serial.println("Pressure 3 (Kpa): ");
  //Serial.println(P3, 3);

  //Serial.println("Voltage 4 (V): ");
  //Serial.println(V4, 3);
  //Serial.println("Pressure 4 (Kpa): ");
  //Serial.println(P4, 3);

  // Write Voltage data 

  //Serial.write(P4);
  Serial.print("V1: ");
  Serial.print(V1,3);
  Serial.println(" Reading");

  //Serial.print("V2: ");
  //Serial.print(V2,3);
  //Serial.println("V");

  //Serial.print("V3: ");
  //Serial.print(V3,3);
  //Serial.println("V");
  
  delay(100);
}
