# Computational SpikerBox (CSB)

The **Computational SpikerBox** (CSB) is a hardware-based lab platform designed to introduce students to **computational neuroscience**. Developed by the engineers at Backyard Brains, the CSB enables students to explore how ion channels of neurons funciton using **mathematical models**.  Studentsi nteract with **physical knobs**” to open, shut, ot operate Na++, K++ amd other Ion Channels.  Optional Addon boards allow studenrts to engage in virtual electrophysiology, determining which neruon-type the electrode is picking up by interactng with various sensors (touch, vibraion, light, heat, etc) — bridging neuroscience theory with hands-on experience.

## Overview

- **What It Does**  
  1. **Hodgkin-Huxley Model**: A spiking neuron model where students can twist knobs (ion channel conductances) to see how poisons and venums can affect the nervous system.  
  2. **Izhikevich + TinyML Model**: An “EPhys Neuron Identification Game” that integrates **machine learning** to classify sensor data and drive a simplified neuron model in real time.

- **Who It’s For**  
  - **Teachers** can run a single CSBox in front of the class, or  
  - **Each student** can have their own CSBox (if resources allow) to dive deeper into computational models, sensor readings, and even machine learning.

- **Educational Goals**  
  - Demonstrate how theoretical neuron equations (Hodgkin-Huxley, Izhikevich) can be **interactively** explored.  
  - Provide a **hands-on** introduction to **machine learning** classification, bridging neuroscience, electronics, and STEM.

---

## Repository Contents

- **`CSBv1.1.ino`**  
  *This is the latest, feature-complete release.* It implements the two main modes (Hodgkin-Huxley for biophysical spiking, and Izhikevich + TinyML for the “Neuron Identification Game”) as described above.
  
- **`ei-empanada-arduino-1.0.X.zip`**  
  *This is the latest Machine Learning header.* Needed for Izhikevich model and Neuron guessing game
  
- **`GGDemoX`**  
  A previous demo-oriented codebase with “bells and whistles” used to showcase earlier features and prototypes.

- **`MLReadin`**  
  A specialized sketch for reading sensor data into Edge Impulse (TinyML) for model training.

- **`ETBiophysical`**  
  An experimental or future direction, converting a Python Colab-style biophysical model into Arduino code.

---

## Team & Acknowledgments

**Team**: Etienne, Chethan, Hatch, Max, with Greg  
**Advice from**: Stan  

> The Izhikevich model portion is adapted from the original paper (2003 IEEE) and informed by [Spikeling v1.1 by T. Baden (Sussex Neuroscience)](www.badenlab.org).

---

## Helpful Links

1. **CSBox High School Lesson Plan**  
   [Google Doc](https://docs.google.com/document/d/10m3qYU1o_Ff_S5vf5cXgXZOqF4JOqBALDKtLj9YVtUo/edit?tab=t.0#heading=h.nyvcnz9wxjju)

2. **Empanada Machine Learning Model**  
   [Edge Impulse Public Project](https://studio.edgeimpulse.com/public/571525/live/impulse/1/learning/keras/7)

3. **Edge Impulse Intro from Grove Article**  
   [Seeed Wiki Page](https://wiki.seeedstudio.com/Wio-Terminal-TinyML-EI-1/)

4. **Edge Impulse Intro from Grove Video**  
   [YouTube Link](https://youtu.be/iCmlKyAp8eQ)

5. **Arduino Library Installation After Model Creation**  
   [Edge Impulse Docs](https://docs.edgeimpulse.com/docs/run-inference/arduino-library)

---

## Usage & Getting Started

1. **Hardware Setup**  
   - Connect your CSBox main board with potentiometers (for Hodgkin-Huxley) and any add-on boards (like “SKIN”) for the ML-based neuron ID game.
   - Make sure you have a compatible microcontroller (e.g., Arduino-type board) and all required sensors (FSR, Temp, Light).

2. **Software & Libraries**  
   - Install the **Arduino IDE** (or PlatformIO).  
   - Ensure you have the **Adafruit NeoPixel** library and any relevant libraries from **Edge Impulse** installed.

3. **Flashing `CSBv1.ino`**  
   - Download both `CSBv1.ino` and `ei-empanada-arduino-1.0.X.zip` 
   - Open `CSBv1.ino` in the Arduino IDE.  
   - Select your board and COM port.
   - Go to Sketch -> Include Library -> Add .ZIP library
   - Add ei-empanada-arduino-1.0.X.zip
   - Compile and upload the sketch.

4. **Running & Observing**  
   - **Mode 1 (Hodgkin–Huxley)**: If no add-on board is detected (attachment sense pin < threshold), you’ll see the spiking behavior controlled by three knobs.  
   - **Mode 2 (Izhikevich + TinyML)**: If the SKIN add-on board is attached (attachment sense pin >= threshold), the sketch runs a classification on sensor data to decide whether to “fire” the neuron model.
   - Open Spike Recorder to see the model play out in real time
   -(Optional debug mode ) Monitor the **Serial** output to see debug info or classification confidence.

5. **Lesson Integration**  
   - Check the [High School Lesson Plan](https://docs.google.com/document/d/10m3qYU1o_Ff_S5vf5cXgXZOqF4JOqBALDKtLj9YVtUo/edit?tab=t.0#heading=h.nyvcnz9wxjju) for ideas on how to structure classroom labs around these models and sensors.  
   - Encourage students to experiment with sensor inputs, observe changes in spiking behaviors, and track their hypotheses versus actual model outcomes.

---

## Future Directions

- **Additional Models**: You can import new or extended neuron models (e.g., Hodgkin–Huxley, heart pacemaker cells).  
- **Deeper ML**: Train your own classifiers with new sensor data (using [Edge Impulse](https://studio.edgeimpulse.com/)).  
- **Advanced Lab Activities**: Introduce noise, bursting phenomena, or combined synaptic inputs for more complex lessons.

---

**License**:  
[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/) – This project is free for educational and non-commercial use.


*Happy Spiking!*
