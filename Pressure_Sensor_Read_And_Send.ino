// Pressure sensor offsets
const float offset_1 = 0.496, offset_2 = 0.474, offset_3 = 0.474, offset_4 = 0.474, offset_5 = 0.474;
// Pressure sensor pins
const int Sensor_Pin_1 = 0, Sensor_Pin_2 = 1, Sensor_Pin_3 = 2, Sensor_Pin_4 = 3, Sensor_Pin_5 = 4;

// Variables for voltage and pressure
float S1, S2, S3, S4, S5;
float V1, V2, V3, V4, V5;
float P1, P2, P3, P4, P5;
char V1_Buf[20], V2_Buf[20], V3_Buf[20], V4_Buf[20], V5_Buf[20];
int Samples = 10;
//float P1, P2, P3, P4;

void Increase_Bits(int Bits){
  // Initialise value sums
  long V1_Sum =0;
  long V2_Sum =0;
  long V3_Sum =0 ;

  // Sample for averaging
  for(int i = 0; i < Samples;i++){
    // Take corresponding samples
    for(int j = 0; j < pow(4,Bits); j++){
    V1_Sum += analogRead(Sensor_Pin_1);
    V2_Sum += analogRead(Sensor_Pin_2);
    V3_Sum += analogRead(Sensor_Pin_3);
    }
  }

  // Average the sums
  V1_Sum = V1_Sum/Samples;
  V2_Sum = V2_Sum/Samples;
  V3_Sum = V3_Sum/Samples;

  // Decimate the position by bit shifting right
  V1_Sum = (V1_Sum >> Bits);
  V2_Sum = (V2_Sum >> Bits);
  V3_Sum = (V3_Sum >> Bits);

  ltoa(V1_Sum, V1_Buf,10);
  ltoa(V2_Sum, V2_Buf,10);
  ltoa(V3_Sum, V3_Buf,10);
}

void setup() {
  // Setup serial communication with a baud rate of 115200 bps
  Serial.begin(115200);
  analogReference(INTERNAL);
  //Serial.println("Sensor readings have begun.");
}

void loop() {
  //Increase_Bits(2);
  // Calculate output voltage. Go from analog to digitial by timing it with the maximum voltage 5V devided by 1024, since we have a 10 bit converter
  //S1 = analogRead(Sensor_Pin_1);// * 5.00/1024;
  //S2 = analogRead(Sensor_Pin_2);// * 5.00/1024;
  //S3 = analogRead(Sensor_Pin_3);// * 5.00/1024;
  S4 = analogRead(Sensor_Pin_4); //* 5.00/1024;
  S5 = analogRead(Sensor_Pin_5);
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
  //Serial.print("S1: ");
  //Serial.print(S1);
  //Serial.println(" Reading");

  //Serial.print("S2: ");
  //Serial.print(S2);
  //Serial.println(" Reading");

  //Serial.print("S3: ");
  //Serial.print(S3);
  //Serial.println(" Reading");

  Serial.print("S4: ");
  Serial.print(S4);
  Serial.println(" Reading");

  Serial.print("S5: ");
  Serial.print(S5);
  Serial.println(" Reading");



  //V1 = S1*1.1/1024;
  //Serial.print("V1: ");
  //Serial.print(V1,8);
  //Serial.println("V");

  //P1 = (V1-offset_1)*250;
  //Serial.print("P1: ");
  //Serial.print(P1,8);
  //Serial.println("kPa");

  
  delay(100);
}
