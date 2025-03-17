#define ENA 6     // Enable pin for motor
#define IN1 4     // Input 1 to L293D
#define IN2 5     // Input 2 to L293D
#define POT A0    // Potentiometer Pin
#define ENCODER 2 // Encoder pin (interrupt)

volatile int pulseCount = 0; // pulse value of encoder
unsigned long prevTime = 0;
float rpm = 0;

// ISR (Interrupt Service Routine) for encoder
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
    // Read value potentiometer
    int potValue = analogRead(POT);
    int motorSpeed = map(potValue, 0, 1023, 0, 255); // Skala PWM

    // Set speed motor
    analogWrite(ENA, motorSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    // Calculate RPM every 1s
    unsigned long currentTime = micros();
    if (currentTime - prevTime >= 1000000) {
        prevTime = currentTime;
        
        float pulsesPerRevolution = 48; // customize specifications encoder pulse in 1 turn
        rpm = (pulseCount / (float)pulsesPerRevolution) * 60; // Convert to RPM
        pulseCount = 0; // Reset pulse count
        
        // show speed value actual
        Serial.print("Speed Motor (RPM): ");
        Serial.println(rpm);
    }
}
