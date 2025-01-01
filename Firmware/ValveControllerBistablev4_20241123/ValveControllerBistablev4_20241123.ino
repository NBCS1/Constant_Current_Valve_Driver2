// Import libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define pins
#define SDA_PIN 19
#define SCL_PIN 18
#define NUM_LEDS 4
#define NUM_BUTTONS 4
int dataPin = 12;      // Set the data pin
int latchPin = 10;
int clockPin = 11;
int ttlPin1 = 20;       // Set TTL pin 1
int ttlPin2 = 21;       // Set TTL pin 2 
int buttonPins[NUM_BUTTONS] = {5, 4, 3, 2}; // Button pins
int ledPins[NUM_LEDS] = {9, 8, 7, 6};      // LED pins
int stateValve[NUM_LEDS] = {0, 0, 0, 0};    // Valve states
int stateButton[NUM_BUTTONS] = {0, 0, 0, 0}; // Button states

// Declare OLED
Adafruit_SSD1306 display(128, 32, &Wire, -1);

// Debounce time
static uint32_t lastDebounceTime = 0;
static uint32_t debounceDelay = 50;

// Variables
int triggerPulseDuration = 500;
unsigned int binaryReset = 0b0000000000000000;
unsigned int dataSend = 0b0000000001010101;
int toggledValve = -1;

//Pulse detection variables
volatile unsigned long pulseStart = 0; // Variable to store the pulse start time
volatile unsigned long pulseEnd = 0; // Variable to store the pulse end time
volatile boolean pulse1=false;
volatile boolean pulse2=false;
unsigned long Start=0;
unsigned long End=0;
unsigned long duration=0;

// Function to update OLED valve status during button overrides
String statePrint(const int index) {
  String output = "v" + String(index + 1) + ":";
  output += stateValve[index] == 1 ? "ON" : "OFF";
  if (index % 2 == 0)
    output += "/";
  return output;
}

// Function to shift out 16 bits
void shiftOut16(int dataPin, int clockPin, unsigned int value) {
  shiftOut(dataPin, clockPin, LSBFIRST, value);  // Send upper 8 bits
  shiftOut(dataPin, clockPin, LSBFIRST, (value >> 8)); // Send lower 8 bits
}

// Function to update shift register
void updateShiftRegister(unsigned int value) {
  digitalWrite(latchPin, LOW);               // Prepare for data transmission
  shiftOut16(dataPin, clockPin, value); 
  // Shift out the 16 bits
  digitalWrite(latchPin, HIGH);              // Latch the data to output it
}

// Function to clear the shift register (set all outputs to LOW)
void clearShiftRegister() {
  updateShiftRegister(binaryReset);  // Shift out all 0s
}

// Update valve states based on the received value
void updateStates(int value) {
  byte valveBits = value & 0xFF;  // Mask to get the last 8 bits

  for (int i = 0; i < NUM_LEDS; i++) {
    // Reverse the order of reading the bits
    byte bitPair = (valveBits >> ((NUM_LEDS - 1 - i) * 2)) & 0b11;

    if (bitPair == 0b01) {
      stateValve[i] = 0;  // Valve is off
    } else if (bitPair == 0b10) {
      stateValve[i] = 1;  // Valve is on
    } else {
      stateValve[i] = -1; // Undefined or invalid state
    }
  }

  // Update stateButton array
  for (int i = 0; i < NUM_BUTTONS; i++) {
    stateButton[i] = stateValve[i];
  }
    for (int i=0; i<NUM_LEDS;i++){
    digitalWrite(ledPins[i], stateValve[i]);
  }
}



// Update valves based on button state
void valveUpdate(int value) {
  uint16_t truncatedValue = value & 0b0000111111111111;// Keep the switches sequence but don't turn the current ON yet otherwise current rush overflow
  updateShiftRegister(truncatedValue);
  delay(1000); //let's the switches open
  updateShiftRegister(value);      // Triggers the right valves combination through switches
  delay(triggerPulseDuration);     // With a x ms pulse 500
  updateStates(value);
  clearShiftRegister(); 
}

unsigned int createBinaryNumber(int toggledValve) {
    unsigned int binaryNumber = 0b11110000;  // Start with the first 8 bits

    // Loop through each LED to set the correct bits
    for (int i = 0; i < NUM_LEDS; i++) {
        unsigned int valveBits = 0b00; // Default to 00

        // Determine the bits to append based on the valve's state

        if (toggledValve == i){
          if (stateValve[i] == 1) {  // Valve is ON
            valveBits = 0b01;  // Set bits to 10 for OFF
        } else if (stateValve[i] == 0) {  // Valve is OFF
            valveBits = 0b10;  // Set bits to 01 for ON
        }}else if (toggledValve != i){
                    if (stateValve[i] == 1) {  // Valve is ON
              valveBits = 0b10;  // Set bits to 10 for ON
          } else if (stateValve[i] == 0) {  // Valve is OFF
              valveBits = 0b01;  // Set bits to 01 for OFF
          }
        }

        // Replace the corresponding pair of bits for the toggled valve
        binaryNumber = (binaryNumber << 2) | valveBits;
    }

    return binaryNumber;
}

void displayConfig(String a,String b,String c){
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(a);
  display.println(b);
  display.println(c);
  display.setCursor(0,0);
  display.display();
}
void pulseStartISR() { pulse1 = true; }// Rising flag
void pulseEndISR() { pulse2 = true; }// Falling flag

// Setup function
void setup() {
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(latchPin, LOW);
  clearShiftRegister();
  dataSend = 0b1111000001010101;
  valveUpdate(dataSend);
  // Attach interrupts to the input pin, pulseStartISR for RISING and pulseEndISR for FALLING
  attachInterrupt(digitalPinToInterrupt(ttlPin1), pulseStartISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ttlPin2), pulseEndISR, FALLING);
  // Open I2C communication with OLED display
  Wire.begin();


  // Set button and LED modes
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // Turn off LEDs
  }


  // Set everything to OFF hardware and software-wise
  clearShiftRegister();
  stateValve[0] = 0; // Initialize valve states
  stateValve[1] = 0;
  stateValve[2] = 0;
  stateValve[3] = 0;
  stateButton[0] = 0; // Initialize button states
  stateButton[1] = 0;
  stateButton[2] = 0;
  stateButton[3] = 0;

  // Initiate display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();
  displayConfig("Hello", "Microfluidic", "User");
  display.display();
}

// Main loop
void loop() {
  // Serial input logic
  if (pulse1){
    pulse1=false;
    Start=micros();
    //Serial.println("yes detected");
  }
  if (pulse2){
    End=micros();
    pulse2=false;
    duration=(End-Start)/1000;
    if (duration > 0) {
      if (duration < 15) { // Reset
        dataSend = 0b1111000001010101;
        valveUpdate(dataSend);
        displayConfig("Config. all off", "Tube 1 -> OFF", "Tube 2 -> OFF");
      } else if (duration < 80) { // Treatment
        dataSend = 0b1111000001101001;
        valveUpdate(dataSend);
        displayConfig("Config. treatment", "Tube 1 -> Trash", "Tube 2 -> Chip");
      } else if (duration < 150) { // All trash
        dataSend = 0b1111000001100110;
        valveUpdate(dataSend);
        displayConfig("Config. all trash", "Tube 1 -> Trash", "Tube 2 -> Trash");
      } else if (duration < 250) { // All chip
        dataSend = 0b111100010011001;
        valveUpdate(dataSend);
        displayConfig("Config. all chip", "Tube 1 -> Chip", "Tube 2 -> Chip");
      } else if (duration < 350) { // Control
        dataSend = 0b1111000010010110;
        valveUpdate(dataSend);
        displayConfig("Config. control", "Tube 1 -> Chip", "Tube 2 -> Trash");
      } else {
        dataSend = 0b1111000001010101;
        valveUpdate(dataSend);
        displayConfig("Trigger duration", "detected", "is wrong");
      }

      display.display();
    }
  }

  // Button logic remains unchanged
  for (int i = 0; i < NUM_BUTTONS; i++) {
    int reading = digitalRead(buttonPins[i]);

    if (reading != stateButton[i]) {
      if (millis() - lastDebounceTime > debounceDelay) {
        stateButton[i] = reading;

        if (stateButton[i] == LOW) {  // Button pressed
          toggledValve = i;  // Track which valve to toggle
          unsigned int binaryNumber = createBinaryNumber(toggledValve);
          valveUpdate(binaryNumber);
          clearShiftRegister();            // Clear the shift register, valves are bistable
                            // Display valve states
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(statePrint(0) + statePrint(1));
        display.println(statePrint(2) + statePrint(3));
        display.println("Manual override");
        display.display();
        }
        lastDebounceTime = millis();
      }

    }

      
  }
   //Control leds according to stateValve

}
