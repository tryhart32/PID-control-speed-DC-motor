// Speed Control PID without library
#define ENA 6       // Enable pin for motor
#define IN1 4       // Input 1 to L293D
#define IN2 5       // Input 2 to L293D
#define POT A0      // Potentiometer for reference speed
#define ENCODER 2   // Encoder (interrupt)

// PID Parameter(need tuning for real condition)
float Kp = 10;  // Gain Proportional
float Ki = 5;   // Gain Integral
float Kd = 1;   // Gain Derivative

volatile int pulseCount = 0; // Encoder Pulse
unsigned long prevTime = 0;
float rpm = 0;
float reference = 0;

// PID Variable 
float error = 0, prevError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

void countPulse() {
    pulseCount++;
}

void setup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(POT, INPUT);
    pinMode(ENCODER, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER), countPulse, RISING);

    Serial.begin(9600);
}

void loop() {
    // Read value potentiometer for set reference RPM
    int potValue = analogRead(POT);
    reference = map(potValue, 0, 1023, -966, 966); // Scale -966(CCW) to 966(CW) RPM

    // Calculated RPM 1s
    unsigned long currentTime = micros();
    if (currentTime - prevTime >= 1000000) {
        prevTime = currentTime;

        float pulsesPerRevolution = 48;
        rpm = (pulseCount / pulsesPerRevolution) * 60; // Convert to RPM
        pulseCount = 0; // Reset counter

        // PID Control
        error = reference - rpm;
        integral += error * 1.0;  // 1 s
        derivative = (error - prevError) / 1.0;
        output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        prevError = error;

        // Convert output PID to PWM
        int pwm = constrain(abs(output), 0, 255); //255 because Arduino has 10 bit resolution PWM
        analogWrite(ENA, pwm);

        // set cw & ccw
        if (output > 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }

        // Print data to Serial Monitor
        Serial.print("Reference: ");
        Serial.print(reference);
        Serial.print(" | RPM: ");
        Serial.print(rpm);
        Serial.print(" | PWM: ");
        Serial.println(pwm);
    }
}
