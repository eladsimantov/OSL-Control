/*
 * Arduino Sketch for reading PWM signal from AS5048 Magnetic Encoder
 * connected to pin 2 (PWM, DIGITAL) of Arduino UNO
 *
 * Circuit Connections:
 * - AS5048 GND to Arduino GND
 * - AS5048 VDD5V to Arduino 5V
 * - AS5048 PWM to Arduino DIGITAL PIN 2.
 * Linux issues temporary connection to board 
 * bash:     sudo chmod a+rw /dev/ttyACM0
 */

unsigned long highTime;
unsigned long lowTime;
float dutyCycle;
float frequency;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT); // Set pin 2 as an input
}

void loop() {
  // Measure the duration of the HIGH and LOW pulses
  highTime = pulseIn(2, HIGH);
  lowTime = pulseIn(2, LOW);

  // Calculate the total period (in microseconds)
  unsigned long totalPeriod = highTime + lowTime;

  // Calculate duty cycle and frequency
  if (totalPeriod > 0) {
    dutyCycle = (float)highTime / totalPeriod * 100.0;
    frequency = 1000000.0 / totalPeriod; // Frequency in Hz (1,000,000 us per second)
  }

  // Print results to Serial Monitor
  Serial.print("High Time: ");
  Serial.print(highTime);
  Serial.print(" us | Low Time: ");
  Serial.print(lowTime);
  Serial.print(" us | Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.print(" % | Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  delay(100); // Small delay before the next reading
}
