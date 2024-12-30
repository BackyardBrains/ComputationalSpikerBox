// Morris Lecar:
// Made for Computational SpikerBox V0.1
// 20. Nov. 2023
// BackyardBrains
// Written: Stanislav Mircic
// 
// Izhikevich:
// Inspiriation and acknowldegment: Spikeling v1.1. By T Baden, Sussex Neuroscience, UK (www.badenlab.org) //
// 2017                                                             //
// Izhikevich model taken from original paper (2003 IEEE)                 //
// 
// Last Updated by: Chethan Magnan
//
// Simulation frequency
// 2.96kHz - with text at baud rate 230400
//
// Inputs:
//        - analog input jack (0-5V)
//
// Outputs:
//        - TTL pulse whenever V crosses 30mV
//        - serial communication @ 2Mbaud (text values of V) 
//
// Potentiometers:
//        Pot R1 - pin 4
//        Pot R2 - pin 5
//        Pot R3 - pin 6
//
//        Analog input 1 - pin 1
//        Analog input 2 - pin 2
//
//        Analog out (LP filtered) 1  - pin 13
//        Analog out (LP filtered) 1  - pin 14
//
/* Edge Impulse ingestion SDK
 * Debugging Notes / Future developments
 Slow tappping is spotty / hard to do consistently, maybe holding for longer is the trick? Sustained?
 Potentiometer hookup, use the ones on the board and just connect it for a 10 value input
 Talk with Alex about designing the board, skin chunk?
 *
 */
//#define USE_SPIKE_RECORDER - Used for an Older Model
#define DEVICE_NAME_CHAR 25

#define SPEAKER A0  
uint8_t		deviceName[DEVICE_NAME_CHAR] = {255, 255, 1, 1, 128, 255, 'H', 'W', 'T', ':', 'M', 'U', 'S', 'C', 'L', 'E', 'S', 'S', ';', 255, 255, 1, 1, 129, 255};
uint8_t outputFrameBuffer[4];

//Time step
double  dt = 1;

 
double minf;
double tau_n;
double ninf;

//initial values 
double   Iapp = 65;
double   V = 0;
double   n = 0;

//Reversal potentials
double   ECa = 120; //mV
double   EK = -84; //mV
double   EL = -60; //mV
double   Cm = 20; //microF/cm2

double   gL = 2; //mS/cm2
double   gK = 8;
double   gKCa = 0.75;

double   V1 = -1.2;
double   V2 = 18;
 
//SNLC - default
double   V3 = 12;
double   V4 = 17.4;
double   phi = 0.067;
double   gCa = 4;

// //Hopf
//double V3 = 2;
//double V4 = 30;
//double phi = 0.04;
//double gCa = 4.4;

// //Homoclinic
//double V3 = 12;
//double V4 = 17.4;
//double phi = 0.23;
//double gCa = 4;

//Variables for Izhikevich
float timestep_ms   = 0.1;
float Array_a[]           =  { 0.1,  0.1, 0.02, 0.02, 0.02,  0.1, 0.02, 0.02}; // time scale of recovery variable u. Smaller a gives slower recovery 
float Array_b[]           =  { 0.15,  0.20, 0.25, 0.20, 0.25, 0.3, 0.25, 0.1}; // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
int   Array_c[]           =  {  -65,   -65,  -55,  -55,  -65, -65, -65, -65}; // after spike reset value
float Array_d[]           =  {  2,   2.0, 0.05,  4.0,  0.05, 2.0, 2.0, 0.05 }; // after spike reset of recovery variable
String Array_e[]          = {"Regular Spiking", "Fast Spiking", "Chattering", "Intrinsically bursting", "Thalamo-Cortical", "Resonator", "Low Threshold Spiking"};
 float Array_PD_decay[]    =  { 0.00005,  0.001, 0.00005,  0.001, 0.00005  }; // slow/fast adapting Photodiode - small numbers make diode slow to decay
float Array_PD_recovery[] =  {   0.001,   0.01,   0.001,   0.01,   0.001  }; // slow/fast adapting Photodiode - small numbers make diode recover slowly
int   Array_PD_polarity[] =  {       1,     -1,      -1,      1,       1  }; // 1 or -1, flips photodiode polarity, i.e. 1: ON cell, 2: OFF cell
// Setup variables required to drive the model
float I_total;           // Total input current to the model
float I_PD;              // Photodiode current
float I_Vm;              // Vm setting current
float I_Synapse;         // Total synaptic current of both synapses
float I_AnalogIn;        // Current from analog Input
float I_Noise;           // Noise current
float Synapse1Ampl;      // Synapse 1 efficacy
float Synapse2Ampl;      // Synapse 2 efficacy
float AnalogInAmpl;      // Analog In efficacy
float NoiseAmpl;         // Added Noise level
long  ModelpreviousMicros   = 0;
float v; // voltage in Iziekevich model
float u; // recovery variable in Iziekevich model
int     VmPotVal       = 0;

/* Includes for TinyML ---------------------------------------------------------------- */
#include <chetjmag-project-2_inferencing.h>


#define FSR 7
#define light 12
static const float features[] = {0.0000, 10.0052, 0.0000, 12.1338, 0.0000, 11.0057, 0.0000, 9.3415, 0.0000, 9.0047, 0.0000, 9.5081, 0.0000, 10.0052, 0.0000, 9.8916, 0.0000, 9.0047, 0.0000, 7.8918, 0.0000, 8.0041, 0.0000, 9.5137, 0.0000, 11.0057, 0.0000, 11.3448, 0.0000, 11.0057, 0.0000, 10.1159, 0.0000, 9.0047, 0.0000, 10.4850, 0.0000, 18.0093, 0.0000, 28.8099, 0.0000, 35.0181, 0.0000, 33.6253, 0.0000, 31.0160, 0.0000, 32.9925, 0.0000, 37.0191, 0.0000, 37.7311, 0.0000, 36.0186, 0.0000, 35.7675, 0.0000, 37.0191, 0.0000, 36.9918, 0.0000, 36.0186, 0.0000, 36.1457, 0.0000, 37.0191, 0.0000, 36.4754, 0.0000, 35.0181, 0.0000, 34.9806, 0.0000, 37.0191, 0.0000, 38.9297, 0.0000, 39.0202, 0.0000, 37.7442
};
float features1[80];
float maxValue = 0;
String category = "0";
int Neuronum = 0;
int NeuronBehaviour = 0; // 0:8 for different modes, cycled by button
  int test = 0;
float intensity = 0;
int NeuronType;

//Lab experiments
  int selection = 0;
  int modelType = 1;
  int stop = 0;
/**
 * @brief      Copy raw feature data in out_ptr
 *             Function called by inference library
 *
 * @param[in]  offset   The offset
 * @param[in]  length   The length
 * @param      out_ptr  The out pointer
 *
 * @return     0
 */
 int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  //O(n) time scale, shouldn't matter, ideally allows for rapid inferencing / Izhk without losing time required for fast
  for(byte i = 0; i<78; i=i+1){
    features1[i] = features1[i+2];
    //Serial.println(features1[i]);
    i = i+1;
    features1[i] = features1[i+2];
    //Serial.println(features1[i]);
  }
  
    delay(15); // Enables Fast?
    features1[78]= analogRead(FSR);
 //   Serial.println(features1[78]); // Debug
    features1[79]= analogRead(light);
   // Serial.println(features1[79]);// Debug
    memcpy(out_ptr, features1 + offset, length * sizeof(float));
    return 0;
}


void print_inference_result(ei_impulse_result_t result);


/**
 * @brief      Arduino setup function
 */

void setup() 
{
  Serial.begin(115200);
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(10);

  // //debug pin
  pinMode(12, OUTPUT);

//TinyML Code
   while (!Serial);
      
  randomSeed(analogRead(0));
  selection = (int)random(0,4); //Random int 1->3, skips fast * 0 and null @ 4
  if(selection == 3){
    selection = 4;
  }

  pinMode(SPEAKER, OUTPUT);   //Testing


//Fill up features array
Serial.println("Creating intial Array");
for(int i = 0; i<80; i++){
  features1[i] = analogRead(FSR);
  i = i+1;
  features1[i] = analogRead(light);
}
Serial.println("Intial Array done");
}
//
// Create a random integer from 0 - 65535
//
unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
}

int inputCounter = 0;
bool somethingArrivedOnSerial = false;

void loop() 
{
  //TinyML
    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
      
        return;
    }
   ei_impulse_result_t result = { 0 };
    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;
    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }
maxValue = 0;
print_inference_result(result);
 Serial.print(selection); // Cheating
  
    Serial.print(", ");
    
    double analogInput = 0;
if(Neuronum== selection){ // Selection for neuron type as rand 
  NeuronBehaviour = 1;
  analogInput = 60;
   stop = 1;
    }
else{
  NeuronBehaviour = 7; //Quiet Neuron
  analogInput = 0;
  stop = 0; //I_Vm + I_Noise 
}

  if(modelType == 0){
  //Morris Lecar
    digitalWrite(12, HIGH);
    Iapp = 0;
    //get current from pot    
    Iapp = Iapp + analogInput;
    minf = 0.5*(1+tanh((V-V1)/V2));
    tau_n = 1.0/cosh((V-V3)/(2*V4));
    ninf = 0.5*(1+tanh((V-V3)/V4));
    V = V + ((-gL*(V-EL) - gK*n*(V-EK) - gCa*minf*(V-ECa)+Iapp)/Cm)*dt;
    n = n + phi*((ninf-n)/tau_n)*dt;
    // while (Serial.available() > 0) 
    // {
    //   // read the incoming byte:
    //   int incomingByte = Serial.read();
    //   somethingArrivedOnSerial = true;
    // }
    // if(somethingArrivedOnSerial)
    // {
    //   somethingArrivedOnSerial = false;
    //   Serial.write(deviceName, DEVICE_NAME_CHAR);
    // }
    //test output
  #ifdef USE_SPIKE_RECORDER
    uint16_t voltage = (V+100)*5;
    //convert data to frame according to protocol
    outputFrameBuffer[0]= (voltage>>7)| 0x80;           
    outputFrameBuffer[1]=  voltage & 0x7F;   
    Serial.write(outputFrameBuffer,2);  
  #else
    Serial.print(V);
  Serial.println("*/");
  #endif
    digitalWrite(12, LOW);
  }

  else{
 //Izhk
 VmPotVal = analogRead(750);
  I_Vm = 40;
  I_Noise = 2.2;
 float I_total = I_Vm*stop + I_Noise; // Add up all current sources
  v = v + 0.5*( timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total ) );
  v = v + 0.5*(timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total));
    u = u + timestep_ms*(Array_a[NeuronBehaviour] * ( 0.1*v - u));
    if (v>=30.0){v=Array_c[NeuronBehaviour]; u+=Array_d[NeuronBehaviour];}
    if (v<=-190) {v=-90.0;} // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
    int AnalogOutValue = (v+90) * 2;
    // compute Izhikevich model
    
  // Ignore synapse, AnalogIn, Array_PD_Polarity,  
  // goal: Map I_VM & noise to potentiometers
     Serial.print(v);
     Serial.print(" ,");
     Serial.print(NeuronType);
   Serial.println(" */"); 

//analogWrite(SPEAKER, v+90);
//analogWrite(SPEAKER, v); //For grove??
analogWrite(A0, v);
  }
}
void print_inference_result(ei_impulse_result_t result) { //Perform classification, this is the inference
    maxValue = 0;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        //ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
       // ei_printf("%.5f\r\n", result.classification[i].value);
        if(result.classification[i].value > maxValue){
          maxValue = result.classification[i].value;
          category = ei_classifier_inferencing_categories[i];
          Neuronum = i;
        }
    }
    
   Serial.print("/* ");        // Frame start sequence  - Serial Studio [/*]
    Serial.print(category);
    Serial.print(", ");
    Serial.print(Neuronum);
    Serial.print(", ");
    Serial.println(maxValue);
    Serial.print(", ");
    
}
