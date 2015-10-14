
// Setup Analog input pins
const int IR_Left_Pin = A0;
const int IR_Right_Pin = A1;

const String leftString = "Left IR Reading: ";
const String rightString = "Right IR Reading: ";

const int IR_Period = 200; // ms

int IR_Left[3] = {0, 0, 0};     // Stores the past 3 Left IR Readings for use in an average
int IR_Right[3] = {0, 0, 0};    // Stores the past 3 Right IR Reading for use in an average
int IR_Value;       // Temporary value to store mesured IR reading
int IR_Left_Avg;  // Used to store left IR average reading
int IR_Right_Avg; // Used to store right IR average reading

unsigned long currentMillis;
unsigned long previousMillis_IR;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

    currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

  if (currentMillis - previousMillis_IR >= IR_Period){
    previousMillis_IR = currentMillis;
    IR_Value = analogRead(IR_Left_Pin);
    
    IR_Left[0] = IR_Left[1];
    IR_Left[1] = IR_Left[2];
    IR_Left[2] = IR_Value;
    
    IR_Value = analogRead(IR_Right_Pin);
    
    IR_Right[0] = IR_Right[1];
    IR_Right[1] = IR_Right[2];
    IR_Right[2] = IR_Value;

    // Calculate the average input
    IR_Left_Avg = (float(IR_Left[0])+float(IR_Left[1])+float(IR_Left[2]))/3;
    IR_Right_Avg = (float(IR_Right[0])+float(IR_Right[1])+float(IR_Right[2]))/3;
    
    Serial.print(leftString + String(IR_Left_Avg) + " " );
    Serial.println(rightString + String(IR_Right_Avg));  

    }

}
