//*****************--CHI CHINH CHO NAY CHO ROBOT MOI------------***********************

#define R_EN1 34
#define L_EN1 35
#define R_PWM1 7
#define L_PWM1 6
#define ENCODER1_CA 21
#define ENCODER1_CB A8


#define R_EN2 32
#define L_EN2 33
#define R_PWM2 5
#define L_PWM2 4
#define ENCODER2_CA 20
#define ENCODER2_CB A9



// // Set motor PWM pins
// #define MOTOR1_PWM_PIN 9   // Timer1 controls pin 9
// #define MOTOR2_PWM_PIN 10  // Timer1 controls pin 10

// void setup() {
//   // Set motor pins as outputs
//   pinMode(MOTOR1_PWM_PIN, OUTPUT);
//   pinMode(MOTOR2_PWM_PIN, OUTPUT);

//   // Initialize Timer1 for fast PWM mode
//   initTimer1();
// }

// void initTimer1() {
//   // Set the Timer1 to 16-bit Fast PWM mode
//   TCCR1A = 0; 
//   TCCR1B = 0; 

//   // Configure Timer1 for fast PWM, non-inverting mode on both pins
//   TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Non-inverting mode on OC1A (pin 9) and OC1B (pin 10)
//   TCCR1A |= (1 << WGM11);                  // Fast PWM, Mode 14 (TOP is ICR1)
//   TCCR1B |= (1 << WGM12) | (1 << WGM13);

//   // Set a prescaler of 8 (gives good control over PWM frequency)
//   TCCR1B |= (1 << CS11);

//   // Set TOP value to control the PWM frequency
//   ICR1 = 40000;  // Set TOP to give a frequency of 50Hz (20ms period)

//   // Set initial duty cycle to 0 (motors off)
//   OCR1A = 0;
//   OCR1B = 0;
// }

// void setMotorSpeed(int motorID, int speed) {
//   if (motorID == 1) {
//     // Motor 1 is controlled by OCR1A
//     OCR1A = map(speed, 0, 255, 0, ICR1); // Map the speed (0-255) to the Timer1 range (0-ICR1)
//   } else if (motorID == 2) {
//     // Motor 2 is controlled by OCR1B
//     OCR1B = map(speed, 0, 255, 0, ICR1); // Map the speed (0-255) to the Timer1 range (0-ICR1)
//   }
// }

// void loop() {
//   int motor1Speed = 150;  // Set motor 1 speed (range 0-255)
//   int motor2Speed = 200;  // Set motor 2 speed (range 0-255)

//   setMotorSpeed(1, motor1Speed);  // Control motor 1
//   setMotorSpeed(2, motor2Speed);  // Control motor 2

//   delay(1000);  // Adjust motor speed every second
// }
