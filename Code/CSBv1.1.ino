/**************************************************************
 *  Project: Computational SpikerBox (CSB)
 *  -----------------------------------------------------------
 *  
 *  This firmware runs two primary modes:
 *
 *  (1) **Hodgkin-Huxley-Serbe MODEL (Mode 1)**:
 *      - Activated when no add-on boards are detected
 *        (attachmentSenseValue < 30).
 *      - Uses 3 knobs on the main CSB to modulate
 *        gNa, gK, and gKCa.
 *      - Demonstrates a simplified “Hodkgin-Huxley–like”
 *        spiking neuron model, with optional “cheating bursts.”
 *
 *  (2) **Neuron Identification Game (Izhikevich + TinyML) (Mode 2)**:
 *      - Activated when the SKIN add-on board is attached
 *        (attachmentSenseValue >= 30).
 *      - Students manipulate sensors on the SKIN board
 *        (SKIN_FSR, Temp, Light).
 *      - An Edge Impulse model classifies which type of neuron
 *        it “thinks” is active, returning probabilities for up
 *        to 5 neuron types (0..4).
 *      - The code maintains a “target” neuron type, chosen
 *        from a random permutation (e.g., [2,4,1,0,3]).
 *      - If the classification probability for the current
 *        target neuron type is above a threshold, the
 *        Izhikevich neuron “fires” (spikes).
 *      - **Joystick** on the main CSB is used to advance to
 *        the *next* target type in the random permutation.
 *        (No button is needed.)
 *
 *  Students do not “enter” guesses in code. They observe
 *  which sensor inputs lead to strong classification
 *  probability—and thus spiking—for the current target neuron.
 *  After finishing the activity, the code can “reveal” the
 *  actual target neuron. They compare that with their notes.
 *
 *  (c) 2025 Backyard Brains / Chethan Magnan
 *  Thanks to Stan, Etienne, Hatch, Max and Greg who also contributed to this code.
 *  Izhikevich model taken from original paper (2003 IEEE) Spikeling v1.1.
 *  By T Baden, Sussex Neuroscience, UK (www.badenlab.org)
 *
 *  Licensed under [CC BY-NC 4.0]
 **************************************************************/

#include <HardwareSerial.h>
#include "driver/uart.h"
#include <Adafruit_NeoPixel.h>
#include <Empanada_inferencing.h>

/**************************************************************
 *          BOARD DETECTION & MODE HANDLING
 **************************************************************/
enum AddOnBoardType {
  BOARD_NONE = 0,  // No add-on: run Morris–Lecar
  BOARD_SKIN,      // SKIN add-on: (5 Neuron Types, 3 sensors)
  BOARD_UNKNOWN
};

/**************************************************************
 *          PIN ASSIGNMENTS & CONSTANTS
 *
 *  We use a consistent naming scheme:
 *    CSB_ for main board pins,
 *    SKIN_ for add-on board pins,
 **************************************************************/


//Spike Recorder Variables
#define TXD0                GPIO_NUM_43
#define RXD0                GPIO_NUM_44
#define DEVICE_NAME_CHAR 25
#define SPIKE_THRESHOLD 30
uint8_t   deviceName[DEVICE_NAME_CHAR] =  {255, 255, 1, 1, 128, 255, 'H', 'W', 'T', ':', 'M', 'U', 'S', 'C', 'L', 'E', 'S', 'S', ';', 255, 255, 1, 1, 129, 255};
uint8_t outputFrameBuffer[4];
#define USE_SPIKE_RECORDER
int32_t           length = 0;
const uart_port_t uart_num = UART_NUM_0;   //UART_NUM_0
uint8_t           receiveBuffer[1000];
bool somethingArrivedOnSerial = false;

/*-------------- Main CSB board --------------*/
#define CSB_Speaker 2
static const int CSB_potPin_gNa = 4;
static const int CSB_potPin_gK = 5;
static const int CSB_potPin_gKCa = 6;

/*-------------- SKIN add-on board --------------*/
#define joystick 1
static const int yAxis = 1;
static const int xAxis = 7;
static const int SKIN_FSR = 10;
static const int SKIN_temperature = 11;
static const int SKIN_light = 12;
static const int SKIN_pullup = 13;
static const int INTERVAL = 25;
static const int SKIN_Attach_Threshold = 30;
static unsigned long lastInterval = 0;


//Neopixel
#define RBGBoard 38
#define RGBNada 18
#define NUMPIXELS 1
Adafruit_NeoPixel CSB_BoardRGB(NUMPIXELS, RBGBoard, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel SKIN_NadaRGB(NUMPIXELS, RGBNada, NEO_GRB + NEO_KHZ800);
static const int DELAYVAL = 500;


//Model Game
float maxValue = 0;
String category = "0";
int Neuronum = 0;
int NeuronType;
int selection = 0;
int stop = 0;
int lightVal = 0;
int tempVal = 0;
int pullupVal = 0;
 //Key: 0 = Fast, 1 = Hold, 2 = Null, 3 = Slow, 4 = Light, 5 = Temp, 6= Everything fires
      //    Meissner  Ruffini   Skipped    Merkle   ???        Free Nerve   ET Req
int NeuronArray[] = {0, 1, 2, 3, 4, 5, 6};
int NeuronR[] = {77, 115, 0, 92,  0, 226, 255};
int NeuronG[] = {17, 253, 0, 203, 0, 0, 255};
int NeuronB[] = {127, 0,  0, 197, 0, 21, 255};
int swap = 0;
int joystickMove = 0;


//Shared Variables
float v = -60;                  // voltage in Iziekevich model
float I_Vm;          // Vm setting current
int startMicros = micros();
float I_Noise;       // Noise current


void checkJoystick(){
#ifdef joystick
int y = analogRead(yAxis);
int x = analogRead(xAxis);
y = abs(y-2000);
x = abs(x-2000);
I_Vm = map(x+y, 0, 4000, 0, 50); // Applied Voltage
joystickMove = I_Vm;
#else
I_Vm = 40; //If no Joystick
#endif
}

/**************************************************************
 *   DETECT BOARD TYPE
 **************************************************************/
AddOnBoardType detectAddOnBoard() {
      pullupVal = analogRead(SKIN_pullup);
  // For now, if pullupVal < 30 => no board, else SKIN.
  if (pullupVal  < SKIN_Attach_Threshold) {
    return BOARD_NONE;
  } else {
    return BOARD_SKIN;
  }
}

/**************************************************************
 *   SPEAKER CLICK FUNCTION
 *   If the membrane voltage is > 0, quickly pulse the speaker.
 **************************************************************/
inline void clickSpeaker(float membraneVoltage) {
  if (membraneVoltage > 0.0f) {
    digitalWrite(CSB_Speaker, HIGH);
    CSB_BoardRGB.setBrightness(250);
    CSB_BoardRGB.show();
    delayMicroseconds(150);
  }
  CSB_BoardRGB.setBrightness(0);
    CSB_BoardRGB.show();
  digitalWrite(CSB_Speaker, LOW);
}


/**************************************************************
 *   MODE 1: Hodgkin-Huxley-Serbe MODEL (when BOARD_NONE)
 *   ---------------------------------------------------------
 *   This is a simplified or “like” model with three knobs:
 *     gNa, gK, gKCa.
 *   The code includes “cheating bursts” for demonstration.
 **************************************************************/


bool biophys = false;
static const int V_MIN = -100.0;  // Minimum voltage (mV)
static const int V_MAX = 150.0;    // Maximum voltage (mV)
static const int STEP_SIZE = 1; // Step size (mV)
static const int TABLE_SIZE = ((int)((V_MAX - V_MIN) / STEP_SIZE) + 1);
bool openET = false;
bool closeET = false;
double lastgNa = 100;
double lastgK= 120;
double lastgKCa = 2;
double gNa = 100.0, gK = 120.0, gKCa = 2.0, gCa = 4.0, gL = 3.0, gH = 0.1;
//double Iapp_t; Replaced with I_Vm
double ENa = 50.0, EK = -84.0, ECa = 120.0, EL = -60.0, EH = -20.0;
double Cm = 20.0;
double dt = 1;
double V1_m = 0.0, V2_m = 18.0, V1_h = -35.0, V2_h = 7.0;
double phi = 0.067, eps = 0.07, mu = 0.02, kCa = 0.0, Iapp = 0.0;
double V = -60.0; // Membrane potential (mV)
double n = 0.0;   // Potassium activation variable
double h = 1.0;   // Sodium inactivation variable
double Ca = 0.0;  // Intracellular calcium concentration
static float m_Vhalf = -40.0;  // midpoint for m
static float m_k     = 10.0;   // slope factor for m
static float h_Vhalf = -60.0;  // midpoint for h
static float h_k     = -7.0;   // slope factor for h (negative slope is common)
static float n_Vhalf = -55.0;  // midpoint for n
static float n_k     = 10.0;   // slope factor for n
static float p_Vhalf = -40.0;  // midpoint for p (KCa or custom gating)
static float p_k     = 20.0;   // slope factor for p
float m_table[TABLE_SIZE];
float h_table[TABLE_SIZE];
// Precompute the lookup tables
float tanh_arduino(float x) {
    float e_pos = exp(x);     // e^x
    float e_neg = exp(-x);    // e^-x
    return (e_pos - e_neg) / (e_pos + e_neg); // tanh(x)
}
void precompute_lookup_tables() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float V = V_MIN + i * STEP_SIZE;
        m_table[i] = 0.5 * (1.0 + tanh_arduino((V - V1_m) / V2_m));
        h_table[i] = 0.5 * (1.0 - tanh_arduino((V - V1_h) / V2_h));
    }
}
// Lookup function for a given voltage
float lookupGatingValue(float V, const float* table) {
    int index = (int)((V - V_MIN) / STEP_SIZE);
    if (index < 0) index = 0;
    if (index >= TABLE_SIZE) index = TABLE_SIZE - 1;
    return table[index];
}
static const int NOISE_TABLE_SIZE = 64;
// Precomputed Gaussian random values (mean ~ 0, std ~ 1)
static const float gaussianNoiseTable[NOISE_TABLE_SIZE] = {
  -1.3329,  0.9826,   0.4843,  -0.1214,  1.2290,  0.0565,  -0.9897,  0.3392,
   0.1113,  1.7751,  -1.1258,  0.6378,   0.2217, -0.5820, -1.7633,  1.4462,
   0.4055,  0.6012,   1.0519, -1.4113,   0.3548, -1.1985,  0.2236,  2.1058,
  -0.7713,  0.7255,   0.1817,  0.9403,   2.3312, -0.0038, -0.6729,  1.0432,
  -1.0027,  0.5204,   0.7997,  0.1635,  -1.6616,  0.9984,  2.2509, -0.2626,
   0.1119, -0.4346,  -1.3451,  1.6985,  -0.8313,  0.3079,   0.0251,  0.5127,
  -1.1362,  1.1857,  -1.9837, -0.5524,   0.6593,  1.0220,   2.4401, -2.1056,
   0.1338, -0.3201,   0.0242, -0.9855,  -0.7412,  1.9028,   0.7843, -0.1909
};
float getRandomNoise(){
  int index = random(NOISE_TABLE_SIZE);
  return gaussianNoiseTable[index];
}
void updateHodgkinHuxleyET(){
 CSB_BoardRGB.clear();
  CSB_BoardRGB.setBrightness(250);
  I_Vm = I_Vm * 8;
  int gNa = map(analogRead(CSB_potPin_gNa), 0, 4095, -200, 400);
  int gK = map(analogRead(CSB_potPin_gK), 0, 4095, 0, 240);
  int gKCa = map(analogRead(CSB_potPin_gKCa), 0, 4095, -6, 10);
  gNa = constrain(gNa, 0, 400);
  gKCa = constrain(gKCa, 0, 10);
  float minf = lookupGatingValue(v, m_table);
  float hinf = lookupGatingValue(v, h_table);
  double dn = phi * (minf - n ) * dt; //
  n += dn;
  n = constrain(n, 0.0, 1.0);
  double dh = phi * (hinf - h ) * dt;
  h += dh;
  h = constrain(h, 0.0, 1.0);
  double z = Ca / (Ca + 1.0);
  double IKCa = gKCa * z * (v - EK);
  double I_Na = gNa * minf * h * (ENa - v); // Sodium current
  double I_K = gK * n * (v - EK);           // Potassium current
  double I_Ca = gCa * minf * (v - ECa);     // Calcium current
  double I_L = gL * (v - EL);               // Leak current
  double I_H = gH * (EH - v);               // H current
  double noise_std = 0.1;
  float dV =  I_Noise + (I_Vm -I_L - I_K - I_Ca - IKCa + I_Na + I_H) / Cm * dt;
  v += dV;


//Cheating, for spontaneous Bursting **********************************************************************************************************************************************
if ((gKCa < 1 ) && (gNa > 100) && (gK > 0) && random(0, 1000) < 5) { // 5% chance per loop iteration
    float burstCurrent = 50.0; // Adjust magnitude for desired bursting effect
    v += burstCurrent; // Add depolarizing burst
}
else if ((gNa > 200) && (gK > 0) && random(0, 1000) < (gNa / 150)) { // chance increases with gNa, 1% with 200, 2-3% with 300+
    float burstCurrent = 50.0; // Adjust magnitude for desired bursting effect
    v += burstCurrent; // Add depolarizing burst
}


  v = constrain(v, -100, 150);
  double dCa = eps * (-mu * I_Ca - kCa * Ca) * dt;
  Ca += dCa;
  Ca = max(Ca, 0.0);
 
  if(lastgNa == 0 || lastgK == 0 || lastgKCa == 0 ){
    closeET = true;
  }


  if (lastgNa > 250 || lastgK > 200 || lastgKCa > 8  ){
    openET = true;
  }
 


  if(openET){
    CSB_BoardRGB.setPixelColor(0, CSB_BoardRGB.Color(0, 0, 255));
    openET = false;
    }
  else if(closeET){
    CSB_BoardRGB.setPixelColor(0, CSB_BoardRGB.Color(255, 0, 0));
    closeET = false;
  }
  else{
    CSB_BoardRGB.setPixelColor(0, CSB_BoardRGB.Color(0, 255, 0));
    }
 lastgNa = gNa;
 lastgK = gK;
 lastgKCa = gKCa;
CSB_BoardRGB.show();
 //Board
#ifdef USE_SPIKE_RECORDER
ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
  length = uart_read_bytes(uart_num, receiveBuffer, length, 10);  
  if(length > 0)    
  {
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    somethingArrivedOnSerial = true;
  }
    //transform voltage to fit in 10 bit uint (0-1024)
    //make it positive (+100) and stretch over full interval (*5)
   
    uint16_t voltage = (v + 100);
    //convert data to frame accdrding to protocol
    outputFrameBuffer[0]= (voltage>>7)| 0x80;          
    outputFrameBuffer[1]=  voltage & 0x7F;  
    if(somethingArrivedOnSerial)
    {
      uart_tx_chars(uart_num, (const char*)outputFrameBuffer, 2);
    }
#else
  Serial.print("/* "); //Ask stan about better setup for BYB spike recorder, with other version / ipads in office
  Serial.print(gNa); //Using Serial Studio because of labels
  Serial.print(", ");
   Serial.print(gK);
  Serial.print(", ");
   Serial.print(gKCa);
  Serial.print(", ");
  Serial.print(v);
  Serial.print(", ");
  Serial.print(I_Vm);
  Serial.println(" */");
#endif
clickSpeaker(v);
}


/**************************************************************
 *   EDGE IMPULSE CLASSIFICATION
 *
 *   We'll store sensor data in a rolling “featureWindow[],”
 *   read them every 25 ms, and pass them to the EI model.
 **************************************************************/
static const float features[] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -7.8220, 10.5494, -25.0513, 36.7031, -63.8421, 93.9126, -151.6706, 244.5046, -485.3774, 1930.3049, 4199.2773, 3390.1628, 3809.2432, 3589.6082, 3764.9958, 3694.0596, 3740.6946, 3714.6919, 3632.8113, 3702.3438, 3440.4651, 3430.6143, 748.5348, -102.6315, 61.3696, -41.3724, 24.8071, -17.7649, 9.2544, -6.7679, 2.2369, -1.9204, -0.2286, 0.0000, -5.2077
};
float features1[40];
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
  /* ///Use if Board Is not set-up
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
    */
    //Use if board is ready for testing


  //O(n) time scale, shouldn't matter, ideally allows for rapid inferencing / Izhk without losing time required for fast
  for (byte i = 0; i < 39; i++) {
    features1[i] = features1[i +1];
  }
  if(millis() > lastInterval + INTERVAL){ //Cool because if millis isn't up yet, just keeps old data
      lastInterval = millis();
      features1[39] = analogRead(SKIN_FSR);
      lightVal = analogRead(SKIN_light);
      //Serial.print("Light Val: "); //Debugging
      //Serial.println(lightVal);
      tempVal = analogRead(SKIN_temperature);
      //Serial.print("Temp Val: ");
      //Serial.println(tempVal); //Debugging
  }


 if (offset + length > sizeof(features1) / sizeof(features1[0])) {
        Serial.println("Error: Out-of-bounds access in raw_feature_get_data");
        return -1; // Return error code
    }
memcpy(out_ptr, features1 + offset, length * sizeof(float));
  return 0;
}

void classifyNeuronType(){
      if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
      }
}
void print_inference_result(ei_impulse_result_t result) {
maxValue = 0; // % Certainty that machine learning selection is correct
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    //ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    // ei_printf("%.5f\r\n", result.classification[i].value);
    if (result.classification[i].value > maxValue) {
      maxValue = result.classification[i].value;
      category = ei_classifier_inferencing_categories[i];
      Neuronum = i;
    }
  }
}

/**************************************************************
 *   UPDATE IZHIKEVICH MODEL
 *   If classification for current target neuron
 *   is above a threshold, we feed a large input, causing spike.
 **************************************************************/
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
float I_Synapse;     // Total synaptic current of both synapses
float I_AnalogIn;    // Current from analog Input
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
float u;                  // recovery variable in Iziekevich model


void updateIzhikevich(){
  CSB_BoardRGB.clear();
  SKIN_NadaRGB.clear();
  CSB_BoardRGB.setBrightness(250);
 
    //Machine Learning, cannot be done separately due to result var
    ei_impulse_result_t result = { 0 }; // the features are stored into flash, and we don't want to load everything into RAM
    //This recieves sensors from empanda for machine learning
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;
    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

  maxValue = 0; //Needed here? Maybe - val is reset in print_inference
  if(joystickMove > 25){
    swap++;
    if(swap > 100){
    swap = 0;
    selection++;
      if(selection ==7){ //Loop back around, selection should be 0, 1, 2, 3 or 4
        selection = 0;
      }
      if(selection == 2){
        selection = 3;
      }
    }
  }
  print_inference_result(result);
  //Key: 0 = Fast, 1 = Hold, 2 = Null, 3 = Slow, 4 = Light, 5 = Temp
    if(tempVal < 1400){ //Ambient room temp is 1700, cold spoon drops to 1200
    //Removed upper limit due to indoor heating in the winter, couldn't get reliable value
      Neuronum = 5;
      category = "Temp";
    }
    if(lightVal > 1200){ //Ambient light is excluded by covering, flash light on should max at 4095
      Neuronum = 4; //Light wins all
      category = "Light";
    }
   
    int green = 0;
    if(Neuronum != 4 || Neuronum != 2 ){
        green = 0;
    }
    else{
      green = 1;
    }
 
CSB_BoardRGB.clear();
SKIN_NadaRGB.clear();
    CSB_BoardRGB.setPixelColor(0, CSB_BoardRGB.Color(NeuronR[NeuronArray[selection]], NeuronG[NeuronArray[selection]], NeuronB[NeuronArray[selection]] ));
    CSB_BoardRGB.show();
    SKIN_NadaRGB.setPixelColor(0, SKIN_NadaRGB.Color(NeuronR[Neuronum], NeuronG[Neuronum] +150 * green, NeuronB[Neuronum]));
    SKIN_NadaRGB.show();
  if(Neuronum == NeuronArray[selection] || NeuronArray[selection] == 6 && Neuronum != 2){ //Correct Guess
  NeuronBehaviour = 1; // Nueron response, Fast Spiking, can be set where NeuronBehvaiour = Selection for different modes
  stop = 1; //Allows for applied voltage
  }
  else{  //Incorrect Guess
  NeuronBehaviour = 7; //Quiet Neurons
  stop = 0;  //Kills any applied Voltage
  }
  int I_autoStim = 40;
float I_total = I_autoStim * stop + I_Noise;  // Add up all current sources
  v = v + 0.5 * (timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total));
  v = v + 0.5 * (timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total));
  u = u + timestep_ms * (Array_a[NeuronBehaviour] * (0.1 * v - u));
  if (v >= 30.0) {
    v = Array_c[NeuronBehaviour];
    u += Array_d[NeuronBehaviour];
  }
  if (v <= -190) { v = -90.0; }  // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
  int AnalogOutValue = (v + 90) * 2;
#ifdef USE_SPIKE_RECORDER
ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
  length = uart_read_bytes(uart_num, receiveBuffer, length, 10);  
  if(length > 0)    
  {
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    uart_tx_chars(uart_num, (const char*)deviceName, DEVICE_NAME_CHAR);
    somethingArrivedOnSerial = true;
  }
    //transform voltage to fit in 10 bit uint (0-1024)
    //make it positive (+100) and stretch over full interval (*5)
   
    uint16_t voltage = (v + 100);
    //convert data to frame accdrding to protocol
    outputFrameBuffer[0]= (voltage>>7)| 0x80;          
    outputFrameBuffer[1]=  voltage & 0x7F;  
    if(somethingArrivedOnSerial)
    {
      uart_tx_chars(uart_num, (const char*)outputFrameBuffer, 2);
    }
#else
   Serial.print("/* ");  // Frame start sequence  - Serial Studio [/*]
  //Serial.println(maxValue);
  //Serial.print(", ");
    Serial.print(category); //Name of Stimulation
  Serial.print(", ");
  Serial.print(Neuronum); //Stimulation Type
  Serial.print(", ");
  //selection = 3;// Temporary setup for testing **************************************************************************************************
Serial.print(NeuronArray[selection]); //Cheating, randomly generated answer
  Serial.print(", ");
  Serial.print(v);
  Serial.print(", ");
  Serial.print(NeuronType);
  Serial.println(" */");
#endif
clickSpeaker(v);
}



/**************************************************************
 *   GLOBALS / INSTANCES
 **************************************************************/
AddOnBoardType currentBoardType  = BOARD_NONE;

/**************************************************************
 *   SETUP
 **************************************************************/
void setup()
{
    Serial.begin(115200);
    while(!Serial);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
  //  Serial.println("Edge Impulse Inferencing Demo");
    CSB_BoardRGB.begin(); // Neopixel
    SKIN_NadaRGB.begin();
    CSB_BoardRGB.setBrightness(120); //max 255, attempt to get both neopixels to be the same color
    SKIN_NadaRGB.setBrightness(200);
  //Spike Recorder Setup
#ifdef USE_SPIKE_RECORDER
  uart_config_t uart_config = {
    .baud_rate = 230400,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,//UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 125,
  };
  // Setup UART buffered IO with event queue
  const int     uart_buffer_size_rx = 1000;
  const int     uart_buffer_size_tx = 32000;
  QueueHandle_t uart_queue;
  uart_driver_delete(uart_num);
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size_rx, uart_buffer_size_tx, 10, &uart_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  // Set UART pins(TX, RX, RTS, CTS)
  //ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD1, RXD1, RTS1, CTS1));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif
   
   
  randomSeed(analogRead(0)); // Random Num, Determines type of Neuron for game and noise in HH model
  selection = (int)random(0, 5);  //Random int for array selection, where do you start?
  if(selection == 2){
        selection = 3;
      }
  //Legacy nonsense, never used by us, from Baden
  if (Array_DigiOutMode[NeuronBehaviour] == 0) {
    //pinMode(DigitalIn1Pin, INPUT); // SET SYNAPSE 1 IN AS INTENDED
  } else {
    //pinMode(DigitalIn1Pin, OUTPUT); // SET SYNAPSE 1 IN AS STIMULATOR OUT CHANNEL
  }
  pinMode(CSB_Speaker, OUTPUT);  //Testing
  pinMode(InferenceTime, OUTPUT);
  pinMode(NeuronJump, OUTPUT); //Would be cool to correct neurons together




 // Serial.println("Creating initial Array");
  for (int i = 0; i < 40; i++) { //40 hz sampling rate, so 40 samples with 1 new per cycle
    features1[i] = analogRead(SKIN_FSR);
  }
  //Serial.println("Initial Array Done");
  precompute_lookup_tables();
  //Serial.println("Precompute Lookup Table done");
  v = V; //Weirdness in HH / ET model working with Izhk.
}


/**
 * @brief      Arduino main function
 */
void loop()
{
  currentBoardType =  detectAddOnBoard();

//Use joystick for either applying stimulation or iterating through neuron types
checkJoystick();
//Both funcs have the same noise
I_Noise =  (getRandomNoise());

if(currentBoardType == BOARD_NONE){
  updateHodgkinHuxleyET();
}
else if (currentBoardType == BOARD_SKIN){
  //Izhk
  delay(5); //**************************************************************************************************************** (V V)
  //Prep neuron classification and check if skin-board is attached
  classifyNeuronType(); 
  updateIzhikevich();
}
//Either HH or Izhk hase been computed, both use v for Voltage
else{
Serial.println("WARNING: Unrecognized board type detected.");
  }   
}

