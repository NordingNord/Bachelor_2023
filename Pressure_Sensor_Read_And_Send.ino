// Pressure sensor offsets
const float offset_1 = 0.474, offset_2 = 0.474, offset_3 = 0.474, offset_4 = 0.474, offset_5 = 0.474;
// Pressure sensor pins
const int Sensor_Pin_1 = 0, Sensor_Pin_2 = 1, Sensor_Pin_3 = 2, Sensor_Pin_4 = 3, Sensor_Pin_5 = 4;

// Variables for voltage and pressure
float V1, V2, V3, V4;
int P1, P2, P3, P4;


void setup() {
  // Setup serial communication with a baud rate of 115200 bps
  Serial.begin(115200);
  Serial.println("Sensor readings have begun.");
}

void loop() {
  // Calculate output voltage. Go from analog to digitial by timing it with the maximum voltage 5V devided by 1024, since we have a 10 bit converter
  V1 = analogRead(Sensor_Pin_1) * 5.00/1024;
  V2 = analogRead(Sensor_Pin_2) * 5.00/1024;
  V3 = analogRead(Sensor_Pin_3) * 5.00/1024;
  V4 = analogRead(Sensor_Pin_4) * 5.00/1024;
  // Calculate water pressure
  P1 = (1-offset_1) * 250;
  P2 = (1-offset_2) * 250;
  P3 = (1-offset_3) * 250;
  P4 = (1-offset_4) * 250;

  // Print pressure and voltages
  Serial.println("Voltage 1 (V): ");
  Serial.println(V1, 3);
  Serial.println("Pressure 1 (Kpa): ");
  Serial.println(P1, 3);

  Serial.println("Voltage 2 (V): ");
  Serial.println(V2, 3);
  Serial.println("Pressure 2 (Kpa): ");
  Serial.println(P2, 3);

  Serial.println("Voltage 3 (V): ");
  Serial.println(V3, 3);
  Serial.println("Pressure 3 (Kpa): ");
  Serial.println(P3, 3);

  Serial.println("Voltage 4 (V): ");
  Serial.println(V4, 3);
  Serial.println("Pressure 4 (Kpa): ");
  Serial.println(P4, 3);

  // Write Pressure data
  Serial.write(P1);
  Serial.write(P2);
  Serial.write(P3);
  Serial.write(P4);

  delay(500);

}
