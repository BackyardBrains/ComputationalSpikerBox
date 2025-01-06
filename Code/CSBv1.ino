/**************************************************************
 *  Project: Computational SpikerBox (CSB)
 *  -----------------------------------------------------------
 *  
 *  This firmware runs two primary modes:
 *
 *  (1) **Morris–Lecar Model (Mode 1)**:
 *      - Activated when no add-on boards are detected 
 *        (attachmentSenseValue < 30).
 *      - Uses 3 knobs on the main CSB to modulate 
 *        gNa, gK, and gKCa.
 *      - Demonstrates a simplified “Morris–Lecar–like” 
 *        spiking neuron model, with optional “cheating bursts.”
 *
 *  (2) **Neuron Identification Game (Izhikevich + TinyML) (Mode 2)**:
 *      - Activated when the SKIN add-on board is attached 
 *        (attachmentSenseValue >= 30).
 *      - Students manipulate sensors on the SKIN board 
 *        (FSR, Temp, Light).
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
 *  Thanks to Stan, Etienne, Hatch, Max and Greg also contributed to this code.
 *  Izhikevich model taken from original paper (2003 IEEE) Spikeling v1.1. 
 *  By T Baden, Sussex Neuroscience, UK (www.badenlab.org)
 *
 *  Licensed under [CC BY-NC 4.0]
 **************************************************************/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <CSB_Skin.h>   // Edge Impulse / TinyML for Skin or "Empanada".

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

/*-------------- Main CSB board --------------*/
// Speaker
static const int CSB_SPEAKER_PIN = 2;

// NeoPixel on CSB (if any)
//static const int CSB_NEOPIXEL_PIN = 38; (Are these needed?)
#define CSB_NUM_PIXELS 0

// Morris–Lecar knobs (example pins for gNa, gK, gKCa)
static const int CSB_KNOB_GNA_PIN  = 4;
static const int CSB_KNOB_GK_PIN   = 5;
static const int CSB_KNOB_GKCA_PIN = 6;

// Joystick
//   We'll assume Y-axis is used for toggling target in Mode 2
//   You can adapt thresholds or use X-axis if you prefer.
static const int CSB_JOYSTICK_Y_PIN = 1;
static const int JOYSTICK_THRESHOLD = 600; // Example threshold
static const int JOYSTICK_CENTER    = 2048; // If it's a 12-bit ADC

/*-------------- Attachment sense pin --------------*/
static const int CSB_ATTACHMENT_SENSE_PIN = 13;

// Threshold for deciding if SKIN is attached
static const int SKIN_ATTACHMENT_THRESHOLD = 30;


/*-------------- SKIN add-on board --------------*/
// Sensors
static const int SKIN_FSR_PIN    = 10;
static const int SKIN_TEMP_PIN   = 11;
static const int SKIN_LIGHT_PIN  = 12;

// NeoPixel on SKIN board
static const int SKIN_NEOPIXEL_PIN = 18;
#define SKIN_NUM_PIXELS 1

/**************************************************************
 *   NEO PIXEL OBJECTS
 **************************************************************/
Adafruit_NeoPixel skinNeoPixel(SKIN_NUM_PIXELS, SKIN_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**************************************************************
 *   GLOBAL TIMING
 **************************************************************/
static const unsigned long SENSOR_UPDATE_INTERVAL_MS = 25;
static unsigned long lastSensorUpdateMillis = 0;

/**************************************************************
 *   DETECT BOARD TYPE
 **************************************************************/
AddOnBoardType detectAddOnBoard(int senseValue) {
  // For now, if senseValue < 30 => no board, else SKIN.
  if (senseValue < SKIN_ATTACHMENT_THRESHOLD) {
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
    digitalWrite(CSB_SPEAKER_PIN, HIGH);
    delayMicroseconds(150);
  }
  digitalWrite(CSB_SPEAKER_PIN, LOW);
}

/**************************************************************
 *   MODE 1: MORRIS–LECAR MODEL (when BOARD_NONE)
 *   ---------------------------------------------------------
 *   This is a simplified or “like” model with three knobs:
 *     gNa, gK, gKCa.
 *   The code includes “cheating bursts” for demonstration.
 **************************************************************/
struct MorrisLecarModel {
  // Ion channel conductances (adjusted by knobs):
  float gNa = 100.0f;
  float gK  = 120.0f;
  float gKCa= 2.0f;

  // Additional constants:
  float gCa = 4.0f;
  float gL  = 3.0f;
  float gH  = 0.1f;

  float eNa = 50.0f;
  float eK  = -84.0f;
  float eCa = 120.0f;
  float eL  = -60.0f;
  float eH  = -20.0f;

  float Cm  = 20.0f;
  float dt  = 1.0f;
  float phi = 0.067f;
  float eps = 0.07f;
  float mu  = 0.02f;
  float kCa = 0.0f;

  // State variables:
  float V     = -60.0f; // Membrane potential
  float n     = 0.0f;   // gating for K
  float h     = 1.0f;   // gating for Na
  float CaIn  = 0.0f;   // intracellular Ca
};

MorrisLecarModel mlModel;

/**
 * @brief Updates the Morris–Lecar model based on knob readings
 *        and random noise. Then toggles speaker if V>0.
 */
void updateMorrisLecar(MorrisLecarModel &ml) {
  // Read knob positions
  int rawNa  = analogRead(CSB_KNOB_GNA_PIN);
  int rawK   = analogRead(CSB_KNOB_GK_PIN);
  int rawKCa = analogRead(CSB_KNOB_GKCA_PIN);

  // Map knob readings to channel conductances
  ml.gNa  = map(rawNa,  0, 4095, 0, 400);
  ml.gK   = map(rawK,   0, 4095, 0, 200);
  ml.gKCa = map(rawKCa, 0, 4095, 0, 10);

  // Example external input
  float userInputV = 40.0f;

  // Light random noise
  float noise = (float)random(-5, 5);

  // Gating placeholders (in a real system, you might do
  // piecewise linear or small LUT for speed)
  float minf = 0.5f;
  float hinf = 0.5f;

  // Update gating variables
  float dn = ml.phi * (minf - ml.n) * ml.dt;
  ml.n += dn; 
  if (ml.n < 0.0f) ml.n=0.0f; 
  if (ml.n > 1.0f) ml.n=1.0f;

  float dh = ml.phi * (hinf - ml.h) * ml.dt;
  ml.h += dh;
  if (ml.h < 0.0f) ml.h=0.0f; 
  if (ml.h > 1.0f) ml.h=1.0f;

  // Currents
  float z      = ml.CaIn / (ml.CaIn + 1.0f);
  float I_KCa  = ml.gKCa * z * (ml.V - ml.eK);
  float I_Na   = ml.gNa  * minf * ml.h * (ml.eNa - ml.V);
  float I_K    = ml.gK   * ml.n * (ml.V - ml.eK);
  float I_Ca   = ml.gCa  * minf * (ml.V - ml.eCa);
  float I_L    = ml.gL   * (ml.V - ml.eL);
  float I_H    = ml.gH   * (ml.eH - ml.V);

  float netCurrent = (userInputV + noise) 
                     - I_L - I_K - I_Ca - I_KCa 
                     + I_Na + I_H;

  float dV = (netCurrent / ml.Cm) * ml.dt;
  ml.V += dV;

  // Optional “cheating bursts”
  if (ml.gKCa == 0 && ml.gNa > 100 && ml.gK > 0 && (random(1000) < 5)) {
    ml.V += 50.0f;
  }

  // Constrain potential
  if (ml.V < -100.0f) ml.V = -100.0f;
  if (ml.V > 150.0f)  ml.V = 150.0f;

  // Update Ca
  float dCa = ml.eps * (-ml.mu*I_Ca - ml.kCa*ml.CaIn) * ml.dt;
  ml.CaIn += dCa; 
  if (ml.CaIn < 0.0f) ml.CaIn = 0.0f;

  // Speaker
  clickSpeaker(ml.V);
}

/**************************************************************
 *   MODE 2: IZHIKEVICH + TINYML FOR “NEURON GAME”
 *   ---------------------------------------------------------
 *   The SKIN board is attached. We read FSR/TEMP/LIGHT,
 *   run an Edge Impulse model, and check the probability
 *   for the *current target neuron type*.
 *   If above a threshold, we feed a high “input” to Izhikevich,
 *   causing it to spike.
 *
 *   We do NOT compare “correct vs. student guess” in code.
 *   The code has a random permutation of neuron types 
 *   (0..4). The “target” is advanced using the joystick 
 *   Y-axis, so we do not need a button.
 **************************************************************/
struct IzhikevichModel {
  float v = -60.0f;
  float u = 0.0f;
  float dt= 0.1f;
};

IzhikevichModel izhModel;

// Example param arrays (0..4) 
// Adjust for your neuron definitions
static float izhA[] = {0.1, 0.1, 0.02, 0.02, 0.02};
static float izhB[] = {0.15,0.20,0.25,0.20,0.25};
static int   izhC[] = {-65, -65, -55, -55, -65};
static float izhD[] = {2.0, 2.0, 0.05,4.0, 0.05};

/**************************************************************
 *   RANDOM TARGET NEURON SEQUENCE
 **************************************************************/
static const int MAX_TARGET_NEURONS = 5; // Types 0..4
int targetNeuronSequence[MAX_TARGET_NEURONS] = {0,1,2,3,4};
int currentTargetIndex    = 0;   // which position in the sequence
int currentTargetNeuron   = 0;   // actual “target” type from the sequence

/**
 * Shuffle the array of possible target neurons
 * using a Fisher-Yates approach for randomness.
 */
void shuffleTargetSequence() {
  for (int i = MAX_TARGET_NEURONS - 1; i > 0; i--) {
    int j = random(i+1); // random from 0..i
    // swap
    int temp = targetNeuronSequence[i];
    targetNeuronSequence[i] = targetNeuronSequence[j];
    targetNeuronSequence[j] = temp;
  }
}

/**
 * Advance to next target neuron in the sequence.
 * If we pass the end, wrap around to 0.
 */
void nextTargetNeuron() {
  currentTargetIndex++;
  if (currentTargetIndex >= MAX_TARGET_NEURONS) {
    currentTargetIndex = 0;
  }
  currentTargetNeuron = targetNeuronSequence[currentTargetIndex];

  // For debugging or teacher
  Serial.print("New Target Neuron => ");
  Serial.println(currentTargetNeuron);
}

/**
 * Reveal the current target neuron type 
 * (Teacher calls this when done).
 */
void revealTargetNeuronType() {
  Serial.print("Current target neuron is: ");
  Serial.println(currentTargetNeuron);
}

/**************************************************************
 *   EDGE IMPULSE CLASSIFICATION
 *
 *   We'll store sensor data in a rolling “featureWindow[],”
 *   read them every 25 ms, and pass them to the EI model.
 **************************************************************/
static float featureWindow[40];
float currentTargetConfidence = 0.0f; // Probability for the current target

/**
 *  Provide data to the EI run_classifier function, shifting 
 *  the sensor window and reading new samples.
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  // Shift left
  for (int i = 0; i < 39; i++) {
    featureWindow[i] = featureWindow[i+1];
  }

  // Each 25 ms, read sensors
  unsigned long now = millis();
  if ((now - lastSensorUpdateMillis) >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMillis = now;

    // FSR from SKIN
    featureWindow[39] = analogRead(SKIN_FSR_PIN);

    // Additional SKIN sensors
    int tmpVal   = analogRead(SKIN_TEMP_PIN);
    int liteVal  = analogRead(SKIN_LIGHT_PIN);

    // Possibly do something with them if needed for debugging
    // e.g., store them in a global or check thresholds.
  }

  // Bounds check
  if (offset + length > 40) {
    Serial.println("Error: raw_feature_get_data out-of-bounds");
    return -1;
  }

  // Copy from the rolling window to out_ptr
  memcpy(out_ptr, &featureWindow[offset], length * sizeof(float));
  return 0;
}

/**
 *  Run the Edge Impulse model, focusing 
 *  on the current target neuron's probability.
 */
void classifyNeuronType() {
  // Original reference features from your code
  static const float referenceFeatures[] = {
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
    -7.8220, 10.5494, -25.0513, 36.7031, -63.8421,
    93.9126, -151.6706, 244.5046, -485.3774, 1930.3049,
    4199.2773, 3390.1628, 3809.2432, 3589.6082, 3764.9958,
    3694.0596, 3740.6946, 3714.6919, 3632.8113, 3702.3438,
    3440.4651, 3430.6143, 748.5348, -102.6315, 61.3696,
    -41.3724, 24.8071, -17.7649, 9.2544, -6.7679,
    2.2369, -1.9204, -0.2286, 0.0000, -5.2077
  };

  // Build signal
  signal_t featuresSignal;
  featuresSignal.total_length = sizeof(referenceFeatures)/sizeof(referenceFeatures[0]);
  featuresSignal.get_data     = &raw_feature_get_data;

  // Do inference
  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR ret = run_classifier(&featuresSignal, &result, false);
  if (ret != EI_IMPULSE_OK) {
    Serial.print("ERR: run_classifier failed (");
    Serial.print(ret);
    Serial.println(")");
    return;
  }

  // Only store the probability of the *current target* type
  currentTargetConfidence = result.classification[currentTargetNeuron].value;
}

/**************************************************************
 *   UPDATE IZHIKEVICH MODEL
 *   If classification for current target neuron 
 *   is above a threshold, we feed a large input, causing spike.
 **************************************************************/
void updateIzhikevich(IzhikevichModel &m) {
  float threshold = 0.5f;  // Adjust to taste
  float userInput = (currentTargetConfidence >= threshold) ? 40.0f : 0.0f;

  // Some small noise
  float noise = 2.2f;
  float totalInput = userInput + noise;

  // Make sure currentTargetNeuron is within 0..4
  int idx = currentTargetNeuron;
  if (idx < 0) idx=0; 
  if (idx > 4) idx=4;

  float a = izhA[idx];
  float b = izhB[idx];
  float c = izhC[idx];
  float d = izhD[idx];

  // 2-step Euler for stability
  for(int step=0; step<2; step++) {
    m.v += 0.5f * (m.dt * (0.04f*m.v*m.v + 5.0f*m.v + 140.0f - m.u + totalInput));
  }

  // Recovery
  m.u += m.dt * (a * (b*m.v - m.u));

  // Spike check
  if (m.v >= 30.0f) {
    m.v = c;
    m.u += d;
  }

  // Constrain extremes
  if(m.v < -190.0f) m.v = -90.0f;

  // Speaker
  clickSpeaker(m.v);
}

/**************************************************************
 *   NEO PIXELS: VISUAL FEEDBACK FOR CURRENT TARGET NEURON
 *   - The SKIN board pixel could show 
 *     the current target neuron’s color
 **************************************************************/
void updateNeoPixels() {
  skinNeoPixel.clear();

  // For instance, represent confidence as brightness on the CSB pixel
  int brightness = (int)(currentTargetConfidence * 255.0f);
  csbNeoPixel.setPixelColor(0, csbNeoPixel.Color(brightness, 0, 255-brightness));

  // Show which neuron index is the target on the SKIN pixel, 
  // by color mapping
  skinNeoPixel.setPixelColor(
    0,
    skinNeoPixel.Color(
      50 * currentTargetNeuron,     // R
      200 - 30 * currentTargetNeuron, // G
      50 * currentTargetNeuron      // B
    )
  );

  skinNeoPixel.show();
}

/**************************************************************
 *   JOYSTICK HANDLER (MODE 2):
 *   - If the joystick Y-axis moves beyond a threshold,
 *     we cycle to the next target neuron.
 *   - You might want to require the user to center it again
 *     to avoid repeated toggles.
 **************************************************************/
void checkJoystickForTargetCycle() {
  static bool wasFar = false; // track if we were past threshold last loop

  int joyVal = analogRead(CSB_JOYSTICK_Y_PIN);  // 0..4095 if 12-bit

  // Distance from center
  int distanceFromCenter = abs(joyVal - JOYSTICK_CENTER);

  bool isPastThreshold   = (distanceFromCenter > JOYSTICK_THRESHOLD);

  // If we just moved past threshold from not being past threshold before,
  // cycle the target.
  if (!wasFar && isPastThreshold) {
    nextTargetNeuron();
  }

  wasFar = isPastThreshold;
}

/**************************************************************
 *   GLOBALS / INSTANCES
 **************************************************************/
AddOnBoardType currentBoardType  = BOARD_NONE;
int attachmentSenseValue         = 0;

/**************************************************************
 *   SETUP
 **************************************************************/
void setup() {
  Serial.begin(115200);
  while(!Serial){ /* wait if needed */ }

  pinMode(CSB_SPEAKER_PIN, OUTPUT);
  pinMode(CSB_ATTACHMENT_SENSE_PIN, INPUT);

  // Initialize the NeoPixels
  csbNeoPixel.begin();
  csbNeoPixel.setBrightness(200);
  skinNeoPixel.begin();
  skinNeoPixel.setBrightness(200);

  // Seed random
  randomSeed(analogRead(0));

  // Shuffle the target sequence once at start
  shuffleTargetSequence();
  currentTargetIndex  = 0; 
  currentTargetNeuron = targetNeuronSequence[currentTargetIndex];

  // Initialize featureWindow for the SKIN-based sensors
  for(int i = 0; i < 40; i++){
    featureWindow[i] = analogRead(SKIN_FSR_PIN);
  }

  // Initialize Izhikevich
  izhModel.v = -60.0f;
  izhModel.u = izhB[0] * izhModel.v;

  // Optional: Print initial target
  Serial.print("Initial target neuron => ");
  Serial.println(currentTargetNeuron);
}

/**************************************************************
 *   LOOP
 **************************************************************/
void loop() {
  // 1) Detect which board is attached
  attachmentSenseValue = analogRead(CSB_ATTACHMENT_SENSE_PIN);
  currentBoardType     = detectAddOnBoard(attachmentSenseValue);

  // 2) Based on board type, run the appropriate model
  if (currentBoardType == BOARD_NONE) {
    // MODE 1: Morris–Lecar
    updateMorrisLecar(mlModel);

  } else if (currentBoardType == BOARD_SKIN) {
    // MODE 2: Izhikevich + TinyML (Neuron Game)

    // (a) Possibly use the joystick to cycle to next target (We will have button on next board`)
    checkJoystickForTargetCycle();

    // (b) Classify sensor data => get confidence for current target
    classifyNeuronType();

    // (c) Update the Izhikevich model => spike if confidence high
    updateIzhikevich(izhModel);

    // (d) Update visual feedback via NeoPixels
    updateNeoPixels();
  } else {
    // If other boards or unknown
    Serial.println("WARNING: Unrecognized board type detected.");
  }

  // You can reduce or remove this delay for higher speed,
  // but be mindful of any extra load from printing, etc.
  // delay(5);
}