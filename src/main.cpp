#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>
#include <bitset>
#include <iostream>
#include <string>
#include <ES_CAN.h>

volatile uint8_t TX_Message[8] = {0}; // Outgoing CAN message
volatile uint8_t RX_Message[8] = {0}; // Latest received CAN message

QueueHandle_t msgInQ;               // Queue for received messages
QueueHandle_t msgOutQ;              // Queue for messages to transmit
SemaphoreHandle_t CAN_TX_Semaphore; // Counting semaphore for CAN TX mailboxes

// Forward declarations of new task functions
void decodeTask(void *pvParameters)
{
  uint8_t msgIn[8];
  while (1)
  {
    xQueueReceive(msgInQ, msgIn, portMAX_DELAY);
    // Copy the received message to RX_Message (no mutex for testing)
    for (int i = 0; i < 8; i++)
    {
      RX_Message[i] = msgIn[i];
    }
    // (Optionally, process the message to play notes.)
  }
}

void CAN_TX_Task(void *pvParameters)
{
  uint8_t msgOut[8];
  while (1)
  {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    // Wait for a mailbox slot to be free.
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

// CAN receive ISR – very short, just reads message and pushes it onto msgInQ.
void CAN_RX_ISR(void)
{
  uint8_t RX_Message_ISR[8];
  uint32_t id;
  CAN_RX(id, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// CAN transmit ISR – gives the semaphore each time a mailbox slot is freed.
void CAN_TX_ISR(void)
{
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// step size definition
// Update the phase step sizes for 12 notes.
// S = (2^32 * f) / 22000, where f = 440 * 2^((i - 9)/12)
// These values are approximate.
const uint32_t stepSizes[] = {
    51043965, // Note 0: ~261.6 Hz
    54176000, // Note 1: ~277.4 Hz
    57309000, // Note 2: ~293.3 Hz
    60752000, // Note 3: ~311.1 Hz
    64300000, // Note 4: ~329.6 Hz
    68154000, // Note 5: ~349.3 Hz
    72130000, // Note 6: ~369.6 Hz
    76490000, // Note 7: ~391.9 Hz
    81000000, // Note 8: ~415.3 Hz
    85900000, // Note 9: 440.0 Hz (A)
    91000000, // Note 10: ~466.2 Hz
    96400000  // Note 11: ~493.9 Hz
};

// Global variable to hold the current phase step size.
volatile uint32_t currentStepSize = 0;

// C++
// Global timer object using TIM1 from stm32duino HardwareTimer library.
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

struct
{
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  int knob3Rotation; // volume control: valid range 0 to 8

  std::bitset<32> get_inputs()
  {
    std::bitset<32> result;
    xSemaphoreTake(mutex, portMAX_DELAY);
    result = inputs;
    xSemaphoreGive(mutex);
    return result;
  }

  void set_inputs(std::bitset<32> new_inputs)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    inputs = new_inputs;
    xSemaphoreGive(mutex);
  }
} sysState;

// C++
void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  int volume = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  // Right-shift the waveform by (8 - volume) for simple log taper volume control
  Vout = Vout >> (8 - volume);
  analogWrite(OUTR_PIN, Vout + 128);
}

void setRow(uint8_t rowIdx)
{
  // Disable the row select enable pin to prevent glitches.
  digitalWrite(REN_PIN, LOW);

  // Set each row select address pin based on rowIdx bits.
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  // Re-enable the row select enable.
  digitalWrite(REN_PIN, HIGH);
}

// Function to read columns from key matrix TODO
std::bitset<4> readCols()
{

  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

// Task to scan keys and update global currentStepSize.
void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t localStepSize = 0;

  // Static variables for knob 3 decoding
  static uint8_t prevKnob3 = 0; // previous state bits {B,A} as a 2‑bit number
  static int lastKnobSign = 0;  // last valid rotation direction
  static bool firstRun = true;  // to initialize prevKnob3 on first iteration

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> inputsTemp;
    // Scan rows 0 to 2 (12 keys total) and row3 for knob 3 rotation.
    for (uint8_t row = 0; row < 4; row++)
    {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> rowInputs = readCols();
      for (uint8_t col = 0; col < 4; col++)
      {
        inputsTemp[row * 4 + col] = rowInputs[col];
      }
    }

    sysState.set_inputs(inputsTemp);

    // Compute local step size from keys 0-11.
    localStepSize = 0;
    for (int note = 0; note < 12; note++)
    {
      // Note pressed when key reads logic 0.
      if (!sysState.get_inputs()[note])
      {
        localStepSize = stepSizes[note];
      }
    }
    // Atomically update currentStepSize.
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);

    // Within scanKeysTask() — after scanning the key matrix.
    static std::bitset<32> prevInputs; // static variable to hold the previous key states

    // Example for keys 0 to 11:
    for (int note = 0; note < 12; note++)
    {
      bool currPressed = !sysState.get_inputs()[note]; // pressed if logic 0
      bool prevPressed = prevInputs[note];
      if (currPressed != prevPressed)
      { // state changed
        if (currPressed)
          TX_Message[0] = 'P';
        else
          TX_Message[0] = 'R';
        TX_Message[1] = 4;    // For example, fixed octave 4
        TX_Message[2] = note; // Note number
        // Send TX_Message via the transmit queue:
        xQueueSend(msgOutQ, (void*)TX_Message, portMAX_DELAY);
      }
      // Save the current state.
      prevInputs[note] = currPressed;
    }

    // --- Decode knob 3 ---
    // Knob 3 inputs are in row 3, columns 0 (A) and 1 (B)
    // Build a 2‑bit value: bit0 = A, bit1 = B
    uint8_t currentKnob3 = ((sysState.get_inputs()[13] ? 1 : 0) << 1) | (sysState.get_inputs()[12] ? 1 : 0);
    // On the first run, initialize prevKnob3.
    if (firstRun)
    {
      prevKnob3 = currentKnob3;
      firstRun = false;
    }
    int delta = 0;
    // Only update rotation when A (bit0) changes.
    if (prevKnob3 == 0 && currentKnob3 == 1 || prevKnob3 == 3 && currentKnob3 == 2)
      delta = 1;
    else if (prevKnob3 == 1 && currentKnob3 == 0 || prevKnob3 == 2 && currentKnob3 == 3)
      delta = -1;
    lastKnobSign = delta;
    // If both bits changed (impossible transition), assume same direction as last valid.
    if (prevKnob3 == 0 && currentKnob3 == 3 || prevKnob3 == 3 && currentKnob3 == 0 || prevKnob3 == 1 && currentKnob3 == 2 || prevKnob3 == 2 && currentKnob3 == 1)
      delta = lastKnobSign;
    // Update knob rotation using atomic store.
    int tempRotation = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    int newRotation = tempRotation + delta;
    newRotation = max(min(newRotation, 8), 0);
    __atomic_store_n(&sysState.knob3Rotation, newRotation, __ATOMIC_RELAXED);
    prevKnob3 = currentKnob3;
  }
}

// Task to update the display.
void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    // Display current phase step size (read atomically)
    char buf[12];
    uint32_t displayStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
    sprintf(buf, "%lu", displayStepSize);
    u8g2.drawStr(2, 10, buf);
    // Also display lower 12 bits of the key matrix input.
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.get_inputs().to_ulong() & 0xFFF, HEX);
    // Also display knob 3 rotation on the next line.
    int knob3 = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
    u8g2.setCursor(2, 30);
    u8g2.print("Knob3: ");
    u8g2.print(knob3);
    // Display latest received CAN message.
    u8g2.setCursor(66, 30);
    u8g2.print((char)RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    u8g2.sendBuffer();

    digitalToggle(LED_BUILTIN); // Blink LED to indicate timing
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Configure timer to trigger the sampleISR() at 22kHz.
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // --- CAN bus initialisation ---
  CAN_Init(true);             // Loopback mode for testing
  setCANFilter(0x123, 0x7FF); // Only accept messages with ID 0x123
  CAN_Start();
  // Create queues. Each item is 8 bytes.
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
  // Create counting semaphore for 3 mailbox slots.
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  // Register CAN TX and RX ISRs.
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_RegisterRX_ISR(CAN_RX_ISR);

  // Create FreeRTOS tasks.
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      1,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  TaskHandle_t displayHandle = NULL;
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, &displayHandle);

  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(decodeTask, "decode", 128, NULL, 1, &decodeHandle);

  TaskHandle_t canTxHandle = NULL;
  xTaskCreate(CAN_TX_Task, "CAN_TX", 128, NULL, 1, &canTxHandle);

  // Create a mutex to protect the shared sysState.inputs.
  sysState.mutex = xSemaphoreCreateMutex();
  sysState.knob3Rotation = 1; // maximum volume (no attenuation)

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
  // static uint32_t next = millis();
  // static uint32_t count = 0;

  // while (millis() < next)
  //   ; // Wait for next interval

  // next += interval;

  // std::bitset<32> inputs; // 32-bit vector to store all key states

  // // Scan the key matrix rows 0 to 2 (only 12 keys)
  // for (uint8_t row = 0; row < 3; row++)
  // {
  //   setRow(row);
  //   delayMicroseconds(3);
  //   std::bitset<4> rowInputs = readCols();
  //   for (uint8_t col = 0; col < 4; col++)
  //   {
  //     inputs[row * 4 + col] = rowInputs[col];
  //   }
  // }

  // // Use a local variable to compute the step size.
  // uint32_t localStepSize = 0;
  // for (int note = 0; note < 12; note++) {
  //   // Keys read as logic 0 when pressed.
  //   if (!inputs[note]) {
  //     localStepSize = stepSizes[note];
  //   }
  // }
  // // Atomically update the global currentStepSize.
  // __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);

  // Update display
  // u8g2.clearBuffer();                 // clear the internal memory
  // u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // // u8g2.drawStr(2, 10, "Helllo World!"); // write something to the internal memory
  // std::string strValue = std::to_string(currentStepSize);
  // u8g2.drawStr(2, 10, strValue.c_str()); // write something to the internal memory
  // u8g2.setCursor(2, 20);
  // u8g2.print(inputs.to_ulong() & 0xFFF, HEX);

  // u8g2.sendBuffer(); // transfer internal memory to the display

  // // Toggle LED
  // digitalToggle(LED_BUILTIN);
}