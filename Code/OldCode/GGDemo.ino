#include <DoublyLinkedList.h>

/* Edge Impulse ingestion SDK
 * Debugging Notes / Future developments
 Slow tappping is spotty / hard to do consistently, maybe holding for longer is the trick? Sustained?
 Potentiometer hookup, use the ones on the board and just connect it for a 10 value input
 Talk with Alex about designing the board, skin chunk?
 *
 */

//Neopixel
#include <Adafruit_NeoPixel.h>


#define RBGBoard 38
#define RGBNada 18
#define NUMPIXELS 1


Adafruit_NeoPixel BoardRGB(NUMPIXELS, RBGBoard, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel NadaRGB(NUMPIXELS, RGBNada, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

/* Includes ---------------------------------------------------------------- */
#include <chetjmag-project-2_inferencing.h>
#define pullup 13
#define Speaker 2
#define FSR 10
#define light 12
static const float features[] = { 0.0000, 10.0052, 0.0000, 12.1338, 0.0000, 11.0057, 0.0000, 9.3415, 0.0000, 9.0047, 0.0000, 9.5081, 0.0000, 10.0052, 0.0000, 9.8916, 0.0000, 9.0047, 0.0000, 7.8918, 0.0000, 8.0041, 0.0000, 9.5137, 0.0000, 11.0057, 0.0000, 11.3448, 0.0000, 11.0057, 0.0000, 10.1159, 0.0000, 9.0047, 0.0000, 10.4850, 0.0000, 18.0093, 0.0000, 28.8099, 0.0000, 35.0181, 0.0000, 33.6253, 0.0000, 31.0160, 0.0000, 32.9925, 0.0000, 37.0191, 0.0000, 37.7311, 0.0000, 36.0186, 0.0000, 35.7675, 0.0000, 37.0191, 0.0000, 36.9918, 0.0000, 36.0186, 0.0000, 36.1457, 0.0000, 37.0191, 0.0000, 36.4754, 0.0000, 35.0181, 0.0000, 34.9806, 0.0000, 37.0191, 0.0000, 38.9297, 0.0000, 39.0202, 0.0000, 37.7442 };
float features1[80];
float maxValue = 0;
String category = "0";
int Neuronum = 0;
////////////////////////////////////////////////////////////////////////////
// Inspiriation and acknowldegment: Spikeling v1.1. By T Baden, Sussex Neuroscience, UK (www.badenlab.org) //
// 2017                                                             //
// Izhikevich model taken from original paper (2003 IEEE)                 //
////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// KEY PARAMETERS TO SET BY USER  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
int nosound = 0;         // 0 default, click on spike + digi out port active. 1 switches both off
int noled = 0;           // 0 default, 1 switches the LED off
int FastMode = 2;        // default 0; if >0, the script is more optimised for speed by removing some of the serial outputs at the
                         // ... end of the script. This will systematically speed up the whole thing, but the system time will no longer be output as the 8th column
                         // ... meaning that the analysis scripts would need to be adjusted to reflect this (i.e. the array entry of the system time, default - column 8).
                         // FastMode = 0: Stores 8 model parameters via serial, runs at ~280 Hz, system time in column 8 (of 8)
                         // FastMode = 1: Stores 4 model parameters via serial, runs at ~390 Hz, system time in column 4 (of 4)
                         // FastMode = 2: Stores 2 model parameters via serial, runs at ~480 Hz, system time in column 2 (of 2)
                         // FastMode = 3: Stores 0 model parameters via serial, runs at ~730 Hz, DOES NOT SEND DATA TO PC!
                         // The next best thing to further increase speed would be to call the several analog.read/write functions
                         // less frequently. If all are disabled, the mode can exceed 1kHz, but then the dials/PD don't work... One compromise
                         // around this would be to call them less frequently. This would give a little extra speed but eventually make the
                         // dials and photodiode feel "sluggish". The latter is currently not implemented
int AnalogInActive = 1;  // default = 1, PORT 3 setting: Is Analog In port in use? Note that this shares the dial with the Syn2 (PORT 2) dial
int Syn1Mode = 1;        // default 1
                         // Syn1Mode = 0: Synapse 1 Port works like Synapse 2, to receive digital pulses as inputs
                         // Syn1Mode = 1: Synapse 1 Port acts as a Stimulus generator, with pulse frequency being controlled by Syn1Dial
                         // Syn1Mode = 2: Synapse 1 Port acts as a Stimulus generator, generating random Noise sequences (for reverse correlation)
                         // Note: this is being read into the Array_DigiOutMode Array below. This can also be manually set for each Mode, if desired by
                         // simply replacing the Syn1Mode entries in this array with 0, 1 or 2

float PD_Scaling = 0.5;                                              // the lower the more sensitive.                       Default = 0.5
int SynapseScaling = 50;                                             // The lower, the stronger the synapse.                Default = 50
int VmPotiScaling = 2;                                               // the lower, the stronger the impact of the Vm poti.  Default = 2
int AnalogInScaling = 2500;                                          // the lower, the stronger the impact of Analog Input. Default = 2500
int NoiseScaling = 10;                                               // the lower, the higher the default noise level.      Default = 10
float Synapse_decay = 0.995;                                         // speed of synaptic decay.The difference to 1 matters - the smaller the difference, the slower the decay. Default  = 0.995
float PD_gain_min = 0.0;                                             // the photodiode gain cannot decay below this value
float timestep_ms = 0.1;                                             // default 0.1. This is the "intended" refresh rate of the model.
                                                                     // Note that it does not actually run this fast as the Arduino cannot execute the...
                                                                     // ...full script at this rate.  Instead, it will run at 333-900 Hz, depending on settings (see top)
                                                                     // set up Neuron behaviour array parameters
int nModes = 5;                                                      // set this to number of entries in each array. Entries 1 define Mode 1, etc..
                                                                     // Izhikevich model parameters - for some pre-tested behaviours from the original paper, see bottom of the script
                                                                     //Array based on each type {RS,    FS,   CH,  IB,   TC,   RZ, LTS, Quiet}
float Array_a[] = { 0.1, 0.1, 0.02, 0.02, 0.02, 0.1, 0.02, 0.02 };   // time scale of recovery variable u. Smaller a gives slower recovery
float Array_b[] = { 0.15, 0.20, 0.25, 0.20, 0.25, 0.3, 0.25, 0.1 };  // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
int Array_c[] = { -65, -65, -55, -55, -65, -65, -65, -65 };          // after spike reset value
float Array_d[] = { 2, 2.0, 0.05, 4.0, 0.05, 2.0, 2.0, 0.05 };       // after spike reset of recovery variable
String Array_e[] = { "Regular Spiking", "Fast Spiking", "Chattering", "Intrinsically bursting", "Thalamo-Cortical", "Resonator", "Low Threshold Spiking" };
float Array_PD_decay[] = { 0.00005, 0.001, 0.00005, 0.001, 0.00005 };            // slow/fast adapting Photodiode - small numbers make diode slow to decay
float Array_PD_recovery[] = { 0.001, 0.01, 0.001, 0.01, 0.001 };                 // slow/fast adapting Photodiode - small numbers make diode recover slowly
int Array_PD_polarity[] = { 1, -1, -1, 1, 1 };                                   // 1 or -1, flips photodiode polarity, i.e. 1: ON cell, 2: OFF cell
int Array_DigiOutMode[] = { Syn1Mode, Syn1Mode, Syn1Mode, Syn1Mode, Syn1Mode };  // PORT 1 setting. 0: Synapse 1 In, 1: Stimulus out, 2: 50 Hz binary noise out (for reverse correlation)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAMME - ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !   Yes                                                                    //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// set up pins

#define PhotoDiodePin A0  // Photodiode
#define LEDOutPin 9       // LED
#define ButtonPin 4       // Push button to switch spike modes
#define VmPotPin A6       // Resting membrane potential


#define SPEAKER A0  // PSYCH!


#define Syn1PotPin A7   // efficacy synapse 1
#define Syn2PotPin A5   // efficacy synapse 2
#define NoisePotPin A3  // scaling of Noise level
//#define DigitalIn1Pin 2  // Synapse 1 Input - expects 5V pulses
#define DigitalIn2Pin 5  // Synapse 2 input - expects 5V pulses
#define AnalogInPin 2    // Analog in- takes 0-5V (positive only)
#define DigitalOutPin 3  // "Axon" - generates 5V pulses
#define AnalogOutPin 11  // Analog out for full spike waveform

#define InferenceTime 42
#define NeuronJump 40
////////////////////////////////////////////////////////////////////////////
// Setup variables required to drive the model
float I_total;       // Total input current to the model
float I_PD;          // Photodiode current
float I_Vm;          // Vm setting current
float I_Synapse;     // Total synaptic current of both synapses
float I_AnalogIn;    // Current from analog Input
float I_Noise;       // Noise current
float Synapse1Ampl;  // Synapse 1 efficacy
float Synapse2Ampl;  // Synapse 2 efficacy
float AnalogInAmpl;  // Analog In efficacy
float NoiseAmpl;     // Added Noise level
long ModelpreviousMicros = 0;
// initialise state variables for different inputs
boolean spike = false;
int buttonState = 0;
int SpikeIn1State = 0;
int SpikeIn2State = 0;
int VmPotVal = 0;
float Syn1PotVal = 0.0;
float Syn2PotVal = 0.0;
float NoisePotVal = 0.0;
float AnalogInPotVal = 0.0;
float AnalogInVal = 0.0;
int PDVal = 0;
int Stimulator_Val = 0;
// for PD smoothing // this is a dirty hack to smooth the photodiode current which in raw form generates ugly noise spikes
int PDVal_Array[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int PD_integration_counter = 0;
float PDVal_smoothed = 0;
float PD_gain = 1.0;
int NeuronBehaviour = 0;  // 0:8 for different modes, cycled by button
int DigiOutStep = 0;      // stimestep counter for stimulator mode
int Stim_State = 0;       // State of the internal stimulator
float v;                  // voltage in Iziekevich model
float u;                  // recovery variable in Iziekevich model
int worldCounter = 100;
bool vExist = false;
int startMicros = micros();
int x = 1;
int lightVal = 0;
////////////////////////////////////////////////////////////////////////////
// SETUP (this only runs once at when the Arduino is initialised) //////////
////////////////////////////////////////////////////////////////////////////
const int TouchPin = 5;
const int ledPin = 6;

// Messing with NeuronBehaviour
int test = 0;
float intensity = 0;
int NeuronType;

//Lab experiments
int selection = 0;

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
  for (byte i = 0; i < 78; i = i + 1) {
    features1[i] = features1[i + 2];
    //Serial.println(features1[i]);
    i = i + 1;
    features1[i] = features1[i + 2];
    //Serial.println(features1[i]);
    /*
    
    i = i+1;
    features1[i] = features1[i+3]; //For temp
    */
  }

  delay(15);  // Enables Fast?
  features1[78] = analogRead(FSR);
  //   Serial.println(features1[78]); // Debug
  features1[79] = analogRead(light);
  // Serial.println(features1[79]);// Debug
  memcpy(out_ptr, features1 + offset, length * sizeof(float));
  return 0;
}


void print_inference_result(ei_impulse_result_t result);


/**
 * @brief      Arduino setup function
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // For ESP32 Dev Board: 921600
  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial)
    ;

  //Random Num
  BoardRGB.begin();
  NadaRGB.begin();
  randomSeed(analogRead(0));
  selection = (int)random(0, 4);  //Random int 1->3, skips fast * 0 and null @ 4
  if (selection == 3) {
    selection = 4;
  }

  /* Circle buffer idea
     DoublyLinkedList<int> list(true);
     for(int i = 0; i<80; i= i+1 ){
      list.insert(features[i]);
     }
*/
  // Serial.println("Edge Impulse Inferencing Demo"); Title, might break serial studio


  pinMode(TouchPin, INPUT);  // Touch capacitor
  pinMode(ledPin, OUTPUT);
  pinMode(AnalogOutPin, OUTPUT);   // 11 "digital" PWM
  pinMode(DigitalOutPin, OUTPUT);  // 3 digital
  pinMode(LEDOutPin, OUTPUT);      // 9 digital PWM
  //pinMode(DigitalIn1Pin, INPUT); // 4 digital
  pinMode(DigitalIn2Pin, INPUT);  // 5 digital
  pinMode(ButtonPin, INPUT);      // 2 digital
  pinMode(PhotoDiodePin, INPUT);  // 0 analog      ***************************************************************************Spot to change for WIO
  pinMode(AnalogInPin, INPUT);    // 5 analog // same as Synapse 2
  pinMode(VmPotPin, INPUT);       // 3 analog
  pinMode(Syn1PotPin, INPUT);     // 7 analog
  pinMode(Syn2PotPin, INPUT);     // 5 analog // this one also controls the Analog In gain!
  pinMode(NoisePotPin, INPUT);    // 6 analog
  if (Array_DigiOutMode[NeuronBehaviour] == 0) {
    //pinMode(DigitalIn1Pin, INPUT); // SET SYNAPSE 1 IN AS INTENDED
  } else {
    //pinMode(DigitalIn1Pin, OUTPUT); // SET SYNAPSE 1 IN AS STIMULATOR OUT CHANNEL
  }


  pinMode(Speaker, OUTPUT);  //Testing

  pinMode(InferenceTime, OUTPUT);

  pinMode(NeuronJump, OUTPUT);

  //Fill up features array
  Serial.println("Creating intial Array");
  for (int i = 0; i < 80; i++) {
    features1[i] = analogRead(FSR);
    i = i + 1;
    features1[i] = analogRead(light);
  }
  Serial.println("Intial Array done");
}

int count = 0;
/**
 * @brief      Arduino main function
 */
void loop() {
  BoardRGB.clear();
  NadaRGB.clear();
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
  //if(worldCounter == 100) // If too fast to read, should be slowed significantly by ML Model
  //{
  VmPotVal = analogRead(750);  // 0:1023, Vm
                               // int sensorValue = digitalRead(TouchPin);
                               /* Previous code from T. Baden, not used currently. 
   Could have students "Apply" their own voltage via a potentiometer  
    I_Vm = 50 * (intensity+1); // Intensity is 0????
    
      I_Vm = (4095 - VmPotVal) * 100 / 4095;
      
      // Middle amount of noise for semi-realisitic graphs
      NoisePotVal = analogRead(0); // 0:1023, Vm 
      NoiseAmpl = -1 * ((NoisePotVal-512) / NoiseScaling);
      if (NoiseAmpl<0) {NoiseAmpl = 0;}
      I_Noise+=random(-NoiseAmpl/2,NoiseAmpl/2);
      I_Noise*=0.9;  
*/
  I_Vm = 40;
  I_Noise = 2.2;

  Serial.print(selection);  // Cheating

  BoardRGB.setPixelColor(0, BoardRGB.Color(250, 50 * selection, 0));
  BoardRGB.show();


  NadaRGB.setPixelColor(0, NadaRGB.Color(250, 50 * Neuronum, 0));
  NadaRGB.show();
  Serial.print(", ");
  if (Neuronum == selection) {  // Selection for neuron type as rand
    NeuronBehaviour = 1;
    stop = 1;
    NadaRGB.setPixelColor(0, NadaRGB.Color(250, 50 * selection, 0));
    NadaRGB.show();

  } else {
    NeuronBehaviour = 7;  //Quiet Neuron
    stop = 0;             //I_Vm + I_Noise
  }

  float I_total = I_Vm * stop + I_Noise;  // Add up all current sources
  v = v + 0.5 * (timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total));
  v = v + 0.5 * (timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total));
  u = u + timestep_ms * (Array_a[NeuronBehaviour] * (0.1 * v - u));
  if (v >= 30.0) {
    v = Array_c[NeuronBehaviour];
    u += Array_d[NeuronBehaviour];
  }
  if (v <= -190) { v = -90.0; }  // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
  int AnalogOutValue = (v + 90) * 2;
  // compute Izhikevich model

  // Ignore synapse, AnalogIn, Array_PD_Polarity,
  // goal: Map I_VM & noise to potentiometers
  Serial.print(v);
  Serial.print(" ,");
  Serial.print(NeuronType);
  Serial.println(" */");

  //analogWrite(SPEAKER, v+90);
  //analogWrite(SPEAKER, v); //For grove??
  if (v > 0) {

    digitalWrite(Speaker, HIGH);
    delay(1);
  }
  digitalWrite(Speaker, LOW);
}

void print_inference_result(ei_impulse_result_t result) {  //Perform classification, this is the inference
  maxValue = 0;
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    //ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    // ei_printf("%.5f\r\n", result.classification[i].value);
    if (result.classification[i].value > maxValue) {
      maxValue = result.classification[i].value;
      category = ei_classifier_inferencing_categories[i];
      Neuronum = i;
    }
  }

  Serial.print("/* ");  // Frame start sequence  - Serial Studio [/*]
  Serial.print(category);
  Serial.print(", ");
  Serial.print(Neuronum);
  Serial.print(", ");
  Serial.println(maxValue);
  Serial.print(", ");
}
