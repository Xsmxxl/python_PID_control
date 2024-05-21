/*
  Voltaje de fuente externa 9.6V @ 1.0A
  RPM Sin Reductor (~9.6V): 1600 RPM
  Pulsos x vuelta: 12
  Indice de cambio: 1/12
  RPM con reducción a (~9.6V): 300 RPM
*/

int signal = 0;
int pinA = 2;
int motor = 3;
int dirA = 8;
int dirB = 7;
volatile long pulseCount = 0;
unsigned long tPrevio = 0;

const int numReadings = 30; // Número de lecturas para el promedio móvil
float readings[numReadings]; // Array para almacenar las lecturas
int readIndex = 0; // Índice de lectura actual
float total = 0; // Suma total de las lecturas
float average = 0;

void setup() {
  pinMode(pinA, INPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pinA), interruptA, RISING);
  Serial.setTimeout(1);
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  unsigned long tActual = millis();
  unsigned long t = tActual - tPrevio;

  if (t >= 10) {
    long pulses = pulseCount;
    pulseCount = 0;
    float rpm = (pulses * 100 * 60 / 12.0);
    
    total = total - readings[readIndex];
    readings[readIndex] = rpm;
    total = total + rpm;
    average = total / numReadings;
    readIndex = (readIndex + 1) % numReadings;
    average = total / numReadings;
    
    if (Serial.available() > 0) {
      signal = Serial.readString().toInt();
      Serial.println(average);
    }
    
    tPrevio = tActual;
  }

  analogWrite(motor, signal);
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, LOW);
}

void interruptA() {
  pulseCount++;
}