#include <math.h>
// Pin assignments for potentiometers
#define potPin_gNa 4
#define potPin_gK 5
#define potPin_gKCa 6
const int speaker = 2;
#define joystick  1
//Spike Recorder
//#define USE_SPIKE_RECORDER - Used for an Older Model
#define DEVICE_NAME_CHAR 25
uint8_t		deviceName[DEVICE_NAME_CHAR] = {255, 255, 1, 1, 128, 255, 'H', 'W', 'T', ':', 'M', 'U', 'S', 'C', 'L', 'E', 'S', 'S', ';', 255, 255, 1, 1, 129, 255};
uint8_t outputFrameBuffer[4];
//Initial Values
double gNa = 100.0, gK = 120.0, gKCa = 2.0, gCa = 4.0, gL = 3.0, gH = 0.1;
double Iapp_t;
double ENa = 50.0, EK = -84.0, ECa = 120.0, EL = -60.0, EH = -20.0;
double Cm = 20.0;
double V1_m = 0.0, V2_m = 18.0, V1_h = -35.0, V2_h = 7.0;
double phi = 0.067, eps = 0.07, mu = 0.02, kCa = 0.0, Iapp = 0.0;
double dt = 0.2; // Time step (ms)
int time_steps = 100; // Number of simulation steps
double noise_std = 0.1;

// Initial states
double V = -60.0; // Membrane potential (mV)
double n = 0.0;   // Potassium activation variable
double h = 1.0;   // Sodium inactivation variable
double Ca = 0.0;  // Intracellular calcium concentration

int counter = 0;
static unsigned long lastTime = 0;
static bool applyCurrent = false;

unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
}
/*
#define TANH_TABLE_SIZE 256
#define TANH_MIN -3.0
#define TANH_MAX 3.0
#define TANH_STEP ((TANH_MAX - TANH_MIN) / (TANH_TABLE_SIZE - 1))

double tanhTable[TANH_TABLE_SIZE];

void initializeTanhTable() {
    for (int i = 0; i < TANH_TABLE_SIZE; i++) {
        double x = TANH_MIN + i * TANH_STEP;
        tanhTable[i] = tanh(x); // Use the standard `tanh` function during setup
    }
}
*/

void setup() {
  Serial.begin(115200); // Initialize serial communication
  pinMode(speaker, OUTPUT);
  //initializeTanhTable();
}
/*
double tanhCustom(double x) {
    if (x <= TANH_MIN) return -1.0; // Below range
    if (x >= TANH_MAX) return 1.0;  // Above range

    int index = (x - TANH_MIN) / TANH_STEP; // Map x to table index
    double fraction = (x - (TANH_MIN + index * TANH_STEP)) / TANH_STEP;

    // Linear interpolation between table[index] and table[index + 1]
    return tanhTable[index] + fraction * (tanhTable[index + 1] - tanhTable[index]);
}

*/
double tanhCustom(double x) { //Remez alg. approximation
    if (x < -3.0) return -1.0;
    else if (x > 3.0) return 1.0;
    else {
        double x2 = x * x;
        return x * (135135 + x2 * 17325) / (135135 + x2 * (62370 + x2 * 3150));
    }
}



void loop() {
  // Read potentiometer values, 350 max, 0 min, 
  int gNa = map(analogRead(potPin_gNa), 0, 4095, 0, 400); //100 middle, 350 max
  double gKCa = map(analogRead(potPin_gKCa), 0, 4095, 0, 10); //2.0 middle
  int gK = map(analogRead(potPin_gK), 0, 4095, 0, 200); //120 middle, 

  double minf = 0.5 * (1 + tanhCustom((V - V1_m) / V2_m)); // Sodium activation
    double hinf = 0.5 * (1 - tanhCustom((V - V1_h) / V2_h)); // Sodium inactivation

    double tau_h = 1.0; // Time constant for h
    double tau_n = 1.0; // Time constant for n

    // Update n and h
    double dn = phi * (minf - n) / tau_n * dt;
    n += dn;
    n = constrain(n, 0.0, 1.0); // Constrain n between 0 and 1

    double dh = phi * (hinf - h) / tau_h * dt;
    h += dh;
    h = constrain(h, 0.0, 1.0); // Constrain h between 0 and 1

    // Update calcium concentration
    double z = Ca / (Ca + 1.0);
    double IKCa = gKCa * z * (V - EK); // Calcium-activated potassium current

    // Calculate ionic currents
    double I_Na = gNa * minf * h * (ENa - V); // Sodium current
    double I_K = gK * n * (V - EK);           // Potassium current
    double I_Ca = gCa * minf * (V - ECa);     // Calcium current
    double I_L = gL * (V - EL);               // Leak current
    double I_H = gH * (EH - V);               // H current

    // Add random noise
    double noise = noise_std * ((double)rng() / RAND_MAX - 0.5);
  
#ifdef joystick
int y = analogRead(1);
int x = analogRead(7);
y = abs(y-2000);
x = abs(x-2000);
Iapp_t = map(x+y, 0, 4000, 0, 500);
#else
if (millis() - lastTime > 10000) { // Adjust interval as needed
    applyCurrent = !applyCurrent;
    lastTime = millis();
}


Iapp_t = applyCurrent ? 300 : 0;
#endif

    // Update membrane potential
    double dV = ((-I_L - I_K - I_Ca - IKCa + I_Na + I_H + Iapp_t) / Cm) * dt + noise;
    V += dV;
    V = constrain(V, -100.0, 150.0); // Constrain voltage to a reasonable range

    // Update calcium concentration
    double dCa = eps * (-mu * I_Ca - kCa * Ca) * dt;
    Ca += dCa;
    Ca = max(Ca, 0.0); // Calcium concentration can't be negative


    // Print voltage to serial (for plotting)
    

  //Serial Studio Outputs
  Serial.print("/* ");
  Serial.print(V);
  Serial.print(", ");
  Serial.print(gNa);
  Serial.print(", ");
  Serial.print(gK);
  Serial.print(", ");
  Serial.print(gKCa);
  Serial.println(" */"); 
analogWrite(3, V);

}
