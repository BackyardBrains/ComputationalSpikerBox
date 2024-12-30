/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


/* Includes ---------------------------------------------------------------- */
#include <chetjmag-project-1_inferencing.h>


#include <Wire.h>
////////////////////////////////////////////////////////////////////////////
// Spikeling v1.1. By T Baden, Sussex Neuroscience, UK (www.badenlab.org) //
// 2017                                                             //
// Izhikevich model taken from original paper (2003 IEEE)                 //
////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////////////////////////////
// KEY PARAMETERS TO SET BY USER  /////////////////////////////////////////    
///////////////////////////////////////////////////////////////////////////
int   nosound         = 0;    // 0 default, click on spike + digi out port active. 1 switches both off
int   noled           = 0;    // 0 default, 1 switches the LED off
int   FastMode        = 2;    // default 0; if >0, the script is more optimised for speed by removing some of the serial outputs at the
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
int   AnalogInActive  = 1;    // default = 1, PORT 3 setting: Is Analog In port in use? Note that this shares the dial with the Syn2 (PORT 2) dial
int   Syn1Mode        = 1;    // default 1
                              // Syn1Mode = 0: Synapse 1 Port works like Synapse 2, to receive digital pulses as inputs
                              // Syn1Mode = 1: Synapse 1 Port acts as a Stimulus generator, with pulse frequency being controlled by Syn1Dial
                              // Syn1Mode = 2: Synapse 1 Port acts as a Stimulus generator, generating random Noise sequences (for reverse correlation)
                              // Note: this is being read into the Array_DigiOutMode Array below. This can also be manually set for each Mode, if desired by
                              // simply replacing the Syn1Mode entries in this array with 0, 1 or 2
                                                                                     
float PD_Scaling      = 0.5;  // the lower the more sensitive.                       Default = 0.5
int   SynapseScaling  = 50;   // The lower, the stronger the synapse.                Default = 50
int   VmPotiScaling   = 2;    // the lower, the stronger the impact of the Vm poti.  Default = 2
int   AnalogInScaling = 2500; // the lower, the stronger the impact of Analog Input. Default = 2500
int   NoiseScaling    = 10;   // the lower, the higher the default noise level.      Default = 10
 float Synapse_decay = 0.995; // speed of synaptic decay.The difference to 1 matters - the smaller the difference, the slower the decay. Default  = 0.995
float PD_gain_min   = 0.0;   // the photodiode gain cannot decay below this value
float timestep_ms   = 0.1;  // default 0.1. This is the "intended" refresh rate of the model.
                            // Note that it does not actually run this fast as the Arduino cannot execute the...
                            // ...full script at this rate.  Instead, it will run at 333-900 Hz, depending on settings (see top)
 // set up Neuron behaviour array parameters
int   nModes = 5; // set this to number of entries in each array. Entries 1 define Mode 1, etc..
   // Izhikevich model parameters - for some pre-tested behaviours from the original paper, see bottom of the script
float Array_a[]           =  { 0.02,  0.02, 0.02, 0.02, 0.02 }; // time scale of recovery variable u. Smaller a gives slower recovery // Change a[1] - Why was it .1????
float Array_b[]           =  { 0.20,  0.20, 0.25, 0.20, -0.1 }; // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
int   Array_c[]           =  {  -65,   -50,  -55,  -55,  -55 }; // after spike reset value
float Array_d[]           =  {  6.0,   2.0, 0.05,  4.0,  6.0 }; // after spike reset of recovery variable
 float Array_PD_decay[]    =  { 0.00005,  0.001, 0.00005,  0.001, 0.00005  }; // slow/fast adapting Photodiode - small numbers make diode slow to decay
float Array_PD_recovery[] =  {   0.001,   0.01,   0.001,   0.01,   0.001  }; // slow/fast adapting Photodiode - small numbers make diode recover slowly
int   Array_PD_polarity[] =  {       1,     -1,      -1,      1,       1  }; // 1 or -1, flips photodiode polarity, i.e. 1: ON cell, 2: OFF cell
 int   Array_DigiOutMode[] =  {Syn1Mode,Syn1Mode,Syn1Mode,Syn1Mode,Syn1Mode}; // PORT 1 setting. 0: Synapse 1 In, 1: Stimulus out, 2: 50 Hz binary noise out (for reverse correlation)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAMME - ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // set up pins
 
#define PhotoDiodePin A0 // Photodiode
#define LEDOutPin 9      // LED
#define ButtonPin 4      // Push button to switch spike modes
#define VmPotPin A6      // Resting membrane potential


#define SPEAKER A0      // PSYCH!


#define Syn1PotPin A7    // efficacy synapse 1
#define Syn2PotPin A5    // efficacy synapse 2
#define NoisePotPin A3   // scaling of Noise level
#define DigitalIn1Pin 2  // Synapse 1 Input - expects 5V pulses
#define DigitalIn2Pin 5  // Synapse 2 input - expects 5V pulses
#define AnalogInPin A2   // Analog in- takes 0-5V (positive only)
#define DigitalOutPin 3  // "Axon" - generates 5V pulses
#define AnalogOutPin 11  // Analog out for full spike waveform

#define InferenceTime 42
#define NeuronJump 40
 ////////////////////////////////////////////////////////////////////////////
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
 // initialise state variables for different inputs
boolean spike = false;
int     buttonState    = 0;
int     SpikeIn1State  = 0;
int     SpikeIn2State  = 0;
int     VmPotVal       = 0;
float   Syn1PotVal     = 0.0;
float   Syn2PotVal     = 0.0;
float   NoisePotVal    = 0.0;
float   AnalogInPotVal = 0.0;
float   AnalogInVal    = 0.0;
int     PDVal          = 0;
int     Stimulator_Val = 0;
 // for PD smoothing // this is a dirty hack to smooth the photodiode current which in raw form generates ugly noise spikes
int PDVal_Array[]         =  {0,0,0,0,0,0,0,0,0,0};
int PD_integration_counter= 0;
float PDVal_smoothed      = 0;
 float PD_gain = 1.0;
int NeuronBehaviour = 0; // 0:8 for different modes, cycled by button
int DigiOutStep = 0;     // stimestep counter for stimulator mode
int Stim_State = 0;      // State of the internal stimulator
float v; // voltage in Iziekevich model
float u; // recovery variable in Iziekevich model
int worldCounter = 100;
bool vExist = false;
 int startMicros = micros();  
   int x = 1;
   int lightVal = 0;
 ////////////////////////////////////////////////////////////////////////////
// SETUP (this only runs once at when the Arduino is initialised) //////////
////////////////////////////////////////////////////////////////////////////
const int TouchPin=5;
const int ledPin=6;

// Messing with NeuronBehaviour
  int test = 0;
float intensity = 0;
static const float features[] = {
   16.7856, 46.0579, 95.7306, 98.5605, 90.0661, 67.6976, 51.8420, 32.3117, 43.4195, 100.4435, 89.0282, 66.5856, 47.7261, 36.0165, 28.8680, 23.6325, 17.7266, 15.0769, 12.6712, 10.1548, 9.9374, 8.7783, 8.7884, 8.8220, 9.3666, 8.4481, 8.3861, 8.6140, 9.0321, 8.7837, 9.3190, 7.7052, 13.9450, 87.7076, 80.4818, 60.9717, 43.6499, 31.8793, 20.6990, 23.3111
    // copy raw features here (for example from the 'Live classification' page)
    // see https://docs.edgeimpulse.com/docs/running-your-impulse-arduino
};


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
 float features[40];
  for(byte i = 0; i<40; i=i+1)
  {
features[i] = analogRead(A6) / 40; // A0 for ESP32

  }
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}


void print_inference_result(ei_impulse_result_t result);


/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200); // For ESP32 Dev Board: 921600
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    while (!Serial);
   // Serial.println("Edge Impulse Inferencing Demo"); Title, might break serial studio


  pinMode(TouchPin, INPUT); // Touch capacitor
  pinMode(ledPin,OUTPUT);
  pinMode(AnalogOutPin, OUTPUT); // 11 "digital" PWM
  pinMode(DigitalOutPin, OUTPUT); // 3 digital
  pinMode(LEDOutPin, OUTPUT); // 9 digital PWM
  pinMode(DigitalIn1Pin, INPUT); // 4 digital
  pinMode(DigitalIn2Pin, INPUT); // 5 digital
  pinMode(ButtonPin, INPUT); // 2 digital
  pinMode(PhotoDiodePin, INPUT); // 0 analog      ***************************************************************************Spot to change for WIO
  pinMode(AnalogInPin, INPUT); // 5 analog // same as Synapse 2
  pinMode(VmPotPin,INPUT); // 3 analog
  pinMode(Syn1PotPin,INPUT); // 7 analog
  pinMode(Syn2PotPin,INPUT); // 5 analog // this one also controls the Analog In gain!
  pinMode(NoisePotPin,INPUT); // 6 analog
   if (Array_DigiOutMode[NeuronBehaviour]==0) {
      pinMode(DigitalIn1Pin, INPUT); // SET SYNAPSE 1 IN AS INTENDED
    } else {
      pinMode(DigitalIn1Pin, OUTPUT); // SET SYNAPSE 1 IN AS STIMULATOR OUT CHANNEL
    }


  pinMode(SPEAKER, OUTPUT);   //Testing

pinMode( InferenceTime, OUTPUT);

pinMode( NeuronJump, OUTPUT);
}

int count = 0;
/**
 * @brief      Arduino main function
 */
void loop()
{
  digitalWrite(InferenceTime, HIGH); // Start of inference time
   // ei_printf("Edge Impulse standalone inferencing (Arduino)\n");


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

NeuronBehaviour = 3;
    // print inference return code
   // ei_printf("run_classifier returned: %d\r\n", res);
   // print_inference_result(result);
if(result.classification[1].value > result.classification[2].value  && result.classification[1].value > result.classification[5].value){ //Fast tapping
NeuronBehaviour = 1;
intensity = result.classification[1].value;
}
else if(result.classification[2].value > result.classification[5].value){ // Slow Tapping
NeuronBehaviour = 2;
intensity = result.classification[2].value;
}
else if(result.classification[5].value > result.classification[2].value){ // Sustained Pressure
NeuronBehaviour = 5;
intensity = result.classification[5].value;
}
else{ //No Pressure
NeuronBehaviour = 3;
intensity = 0;
}    

digitalWrite(InferenceTime, LOW); // End of inference time
   // int sensorValue = digitalRead(TouchPin);
     I_Vm = 50 * intensity;
     
      // read analog In potentiometer to set Analog in and Noise scaling
      NoisePotVal = analogRead(750); // 0:1023, Vm
      NoiseAmpl = -1 * ((NoisePotVal-512) / NoiseScaling);
      if (NoiseAmpl<0) {NoiseAmpl = 0;}
      I_Noise+=random(-NoiseAmpl/2,NoiseAmpl/2);
      I_Noise*=0.9;  
     
   
   
    // compute Izhikevich model
//if(worldCounter == 100) // If too fast to read, should be slowed significantly by ML Model
//{

    float I_total = I_Vm + I_Noise; // Add up all current sources
    v = v + timestep_ms*(0.04 * v * v + 5*v + 140 - u + I_total);
    u = u + timestep_ms*(Array_a[NeuronBehaviour] * ( Array_b[NeuronBehaviour]*v - u));
    if (v>=30.0){v=Array_c[NeuronBehaviour]; u+=Array_d[NeuronBehaviour];}
    if (v<=-190) {v=-90.0;} // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
    int AnalogOutValue = (v+90) * 2;
  // Ignore synapse, AnalogIn, Array_PD_Polarity,  
  // goal: Map I_VM & noise to potentiometers

   Serial.print("/* ");        // Frame start sequence  [/*]
     Serial.print(v);
     Serial.print(" ,");
     Serial.print(NeuronBehaviour);
   Serial.println(" */"); 

digitalWrite(SPEAKER, v*(-1));
// Fake clicks for audio
  //worldCounter = 0;
  //}
//worldCounter++;
 

}


void print_inference_result(ei_impulse_result_t result) {
/*    IMAGE Classification, not for this project
    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);


    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
               
    }


  */
    // Print the prediction results (classification)
//# 
/*
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }


//#endif
    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
*/

}

