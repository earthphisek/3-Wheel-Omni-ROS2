
#include <ESP32Encoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <QuickPID.h>
#include <PID_v2.h>
//motorA
const int outputA_A = 5;
const int outputB_A = 18;

const int InA_motorA = 23;
const int InB_motorA = 22;
const int PWM_motorA = 21;

//motorB
const int outputA_B = 16;
const int outputB_B = 17;

const int InA_motorB = 33;
const int InB_motorB = 25;
const int PWM_motorB = 26;

//motorC
const int outputA_C = 15;
const int outputB_C = 19;

const int InA_motorC = 13;
const int InB_motorC = 14;
const int PWM_motorC = 27;

long First_encoder;
long Second_encoder;
long Third_encoder;
long deltaCount;
unsigned long last_time;
double test_time;

int M1A;
int M1B;
int PWM1;

double lastError;
static double cumError_A;
static double cumError_B;
static double cumError_C;
double rateError;
//double real_PID;

//find_distance
double dis;

double M_1;
double M_2;
double M_3;

double U1A;
double U2C;
double U3B;

double count;

double PWM;
double volt_required;

float cmd;
static float x, y, rotation;
int32_t test_list[3];
#define LED_PIN 2  // Define the GPIO pin for the LED

ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

#define MAX_DATA_LEN 50  // Maximum length of the received data
#define ARRAY_SIZE 3     // Size of the array to store the data
char receivedData[50];   // Buffer to store received data
int dataArray[3];        // Array to store parsed data



// Setpoints for PID control
double Setpoint_A = 0;
double Setpoint_B = 0;
double Setpoint_C = 0;
// quickPID mypid()
// QuickPID motorPID_A(&First_encoder,&PWM_motorA , &Setpoint_A);
// QuickPID motorPID_B(&Second_encoder,&PWM_motorB , &Setpoint_B);
// QuickPID motorPID_C(&Third_encoder,&PWM_motorC , &Setpoint_C);



void change_command_to_omega(double x, double y, double z, double First_encoder, double Second_encoder, double Third_encoder, double deltaT) {
  // P'NOOK
  M_1 = y + 0.295 * z;
  M_2 = 0.866 * x - 0.5 * y + 0.295 * z;
  M_3 = -0.866 * x - 0.5 * y + 0.295 * z;

  M_1 = M_1 * 26;
  M_2 = M_2 * 26;
  M_3 = M_3 * 26;

  change_to_count(M_1, First_encoder, deltaT, "A");
  change_to_count(M_2, Second_encoder, deltaT, "B");
  change_to_count(M_3, Third_encoder, deltaT, "C");
}

void change_to_count(double omega, double encoder, double deltaT, String Which_motor) {
  int delta_time = 10;          // millisec
  int count_per_second = 1000;  //from encoder
  int sec_to_millisec = 1000;   //sec_to_millisec
  double Big_round = 4.6;       //cm
  double small_round = 0.63;    //cm
  double pi = 3.1416;

  if (Which_motor == "A") {
    count_per_second = 600;
  }

  count = (omega * delta_time * count_per_second * Big_round) / (sec_to_millisec * small_round * 2 * pi);
  // PID(count, encoder, deltaT, Which_motor);
}

// void PID(QuickPID& motorPID, double Setpoint_count, double encoder, String Which_motor) {
//   double real_PID
//   motorPID.Compute();
//   motorPID.SetTunings(Kp, Ki, Kd);
//   real_PID = motorPID.GetOutput();
//   motor(real_PID, 24, Which_motor);
// }



void PID(double Setpoint_count, double encoder, double elapsedTime, String Which_motor) {
  double real_PID_A, real_PID_B, real_PID_C;
  float Kp = 1.2;  //0.6 ///0.2 previous
  float Ki = 0.9;  //0.45 //1.5 previous
  float Kd = 0;

  
  // Serial.print(error);
  if (Which_motor == "A") {
    double error = Setpoint_count - encoder;
    // Serial.println("in A");
    cumError_A += error * elapsedTime;
    real_PID_A = (Kp * error + Ki * cumError_A);  // in VOLT
    // Serial.print(real_PID_A);
    // Serial.print(",");
    motor(real_PID_A, 24, Which_motor);
  } 
  if (Which_motor == "B") {
    double error = Setpoint_count - encoder;
    // Serial.print("in B");
    cumError_B += error * elapsedTime;
    real_PID_B = (Kp * error + Ki * cumError_B);  // in VOLT
    // Serial.print(real_PID_B);
    // Serial.print(",");
    motor(real_PID_B, 24, Which_motor);
  } 
  if (Which_motor == "C") {
    double error = Setpoint_count - encoder;
    // Serial.println("in C");
    cumError_C += error * elapsedTime;
    real_PID_C = (Kp * error + Ki * cumError_C);  // in VOLT
    // Serial.println(real_PID_C);
    motor(real_PID_C, 24, Which_motor);
  }
}

void motor(double volt, double maxVoltage, String Which_motor) {
  if (Which_motor == "A") {
    M1A = InA_motorA;
    M1B = InB_motorA;
    PWM1 = PWM_motorA;
  }
  if (Which_motor == "B") {
    M1A = InA_motorB;
    M1B = InB_motorB;
    PWM1 = PWM_motorB;
  }
  if (Which_motor == "C") {
    M1A = InA_motorC;
    M1B = InB_motorC;
    PWM1 = PWM_motorC;
  }

  if (volt >= maxVoltage) {
    volt = maxVoltage;
  } else if (volt <= -maxVoltage) {
    volt = -maxVoltage;
  }

  if (volt > -30 && volt < 30) {
    // Serial.print("NONE");
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    analogWrite(PWM1, 0);
  }

  if (volt > 0) {
    double pwm = (volt / 24.0) * 255.0;
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
    analogWrite(PWM1, pwm);
  } else if (volt < 0) {
    double pwm = -(volt / 24.0) * 255.0;
    digitalWrite(M1A, HIGH);
    digitalWrite(M1B, LOW);
    analogWrite(PWM1, pwm);
  }

  else {
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, LOW);
    analogWrite(PWM1, 0);
  }
}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  delay(1000);
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(outputA_A, outputB_A);   // current use
  encoder2.attachHalfQuad(outputA_B, outputB_B);  // no input
  encoder3.attachHalfQuad(outputA_C, outputB_C);  // no input

  analogWriteFrequency(20000);

  //motorA
  pinMode(outputA_A, INPUT);
  pinMode(outputB_A, INPUT);

  pinMode(InA_motorA, OUTPUT);
  pinMode(InB_motorA, OUTPUT);
  pinMode(PWM_motorA, OUTPUT);

  //motorB
  pinMode(outputA_B, INPUT);
  pinMode(outputB_B, INPUT);

  pinMode(InA_motorB, OUTPUT);
  pinMode(InB_motorB, OUTPUT);
  pinMode(PWM_motorB, OUTPUT);

  //motorC
  pinMode(outputA_C, INPUT);
  pinMode(outputB_C, INPUT);

  pinMode(InA_motorC, OUTPUT);
  pinMode(InB_motorC, OUTPUT);
  pinMode(PWM_motorC, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  delay(500);
}


void loop() {
  //
  // while (!Serial.available());
  // if (Serial.available() > 0) {
  //   cmd = Serial.readString().toFloat();
  //   // cmd_sub[0] = cmd.subString(4,)
  //   Serial.print(cmd);
  // }

  if (Serial.available() > 0) {  // Check if data is available to read
    // Read the incoming data into the buffer
    int bytesRead = Serial.readBytesUntil('\n', receivedData, MAX_DATA_LEN);
    receivedData[bytesRead] = '\0';  // Null-terminate the string
  // ESP_LOGE("Set Point", "%s", receivedData);
    // Parse the received data and store it in the array
    int count = 0;
    char* ptr = strtok(receivedData, ",");
    while (ptr != NULL && count < ARRAY_SIZE) {
      dataArray[count] = atoi(ptr);  // Convert string to integer and store in the array
      ptr = strtok(NULL, ",");
      count++;
    }
    // ESP_LOGE("Set Point", "A:%d, B:%d, C:%d", dataArray[0],dataArray[1],dataArray[2]);
  }
  if (millis() - last_time >= 10) {

    last_time = millis();
    First_encoder = encoder.getCount();
    test_list[0] = First_encoder;
    encoder.clearCount();

    Second_encoder = encoder2.getCount();
    test_list[1] = Second_encoder;
    encoder2.clearCount();

    Third_encoder = encoder3.getCount();
    test_list[2] = Third_encoder;
    encoder3.clearCount();
    // Serial.println(cmd);
    // Serial.print(test_list[0]);
    // Serial.print(",");
    // Serial.print(test_list[1]);
    // Serial.print(",");
    // Serial.println(test_list[2]);
    // change_command_to_omega( cmd , 0.0 , 0.0 ,First_encoder ,Second_encoder ,Third_encoder, 0.01);
    
    PID(dataArray[0], First_encoder, 0.01, "A");
    PID(dataArray[1], Second_encoder, 0.01, "B");
    PID(dataArray[2], Third_encoder, 0.01, "C");
    // PID(motorPID_A, Setpoint_A, First_encoder, "A");
    // PID(motorPID_B, Setpoint_B, Second_encoder, "B");
    // PID(motorPID_C, Setpoint_C, Third_encoder, "C");
  }
}
