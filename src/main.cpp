#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>
#include <bitset>
#include <iostream>
#include <string>
#include <ES_CAN.h>
#include <Osc3x.h>
#include <KeyPressBuffer.h>
#include <atomic>
#include <song.h>

extern uint8_t autoDetection();

std::vector<uint8_t> songNotes[8];
std::atomic<uint8_t> songIndex;

enum EXE_Mode
{
  SELECT_PLAYING_STATE,
  NORMAL,
  SELECT_TEACHING_STATE,
  TEACHING,
  PLAYING
};

struct
{
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  SemaphoreHandle_t rx_meg_mutex;
  std::atomic<bool> isReceiver = false; // we define the receiver as the leftmost board
  std::atomic<uint8_t> location = 0;    //
  std::atomic<uint8_t> octave = 0;
  std::atomic<EXE_Mode> mode = NORMAL;
  std::atomic<uint8_t> song_index = 0;
  std::atomic<bool> west_record;
  std::atomic<bool> east_record;
  std::atomic<uint8_t> page_column_index[3]{0, 0, 0};
  std::atomic<uint8_t> volume = 1;
} sysState;
#define sysMutexAcquire() xSemaphoreTake(sysState.mutex, portMAX_DELAY);
#define sysMutexRelease() xSemaphoreGive(sysState.mutex);
#define sysMutexAcquireRx() xSemaphoreTake(sysState.rx_meg_mutex, portMAX_DELAY);
#define sysMutexReleaseRx() xSemaphoreGive(sysState.rx_meg_mutex);

// assume that 1 lower lower will be 1
const uint8_t butterfly[] = {
    HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N5, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N3, 1)), HALF_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N5, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_NOTE(NOTE_TRANSHFER(N3, 0)), HALF_NOTE(NOTE_TRANSHFER(N2, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 0)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 0)), HALF_NOTE(NOTE_TRANSHFER(N7, 0)), HALF_NOTE(NOTE_TRANSHFER(N6, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N2, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N7, -1)), HALF_NOTE(NOTE_TRANSHFER(N6, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N7, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N6, 1)), HALF_NOTE(NOTE_TRANSHFER(N7, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), NOTE_TRANSHFER(N5, 1), 3, QUATER_NOTE(NOTE_TRANSHFER(N6, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N4_, 1)), HALF_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N4_, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), NOTE_TRANSHFER(N5, 1), 3, QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N2, -1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N5, 1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N4, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N3, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_NOTE(NOTE_TRANSHFER(N5, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N7, 0)), HALF_NOTE(NOTE_TRANSHFER(N2, 1)), WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N6, 0)), HALF_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), WHOLE_NOTE(NOTE_TRANSHFER(N3, -1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N5, 0)), QUATER_NOTE(NOTE_TRANSHFER(N6, 0)), QUATER_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -2)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK};

// assume that 1 lower lower will be 1
const uint8_t canon[] = {
    HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N5, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N3, 1)), HALF_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N5, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_NOTE(NOTE_TRANSHFER(N3, 0)), HALF_NOTE(NOTE_TRANSHFER(N2, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 0)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, 0)), HALF_NOTE(NOTE_TRANSHFER(N7, 0)), HALF_NOTE(NOTE_TRANSHFER(N6, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N2, 0)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N7, -1)), HALF_NOTE(NOTE_TRANSHFER(N6, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_STOP, HALF_NOTE(NOTE_TRANSHFER(N7, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N6, 1)), HALF_NOTE(NOTE_TRANSHFER(N7, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), NOTE_TRANSHFER(N5, 1), 3, QUATER_NOTE(NOTE_TRANSHFER(N6, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N4_, 1)), HALF_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N4_, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), NOTE_TRANSHFER(N5, 1), 3, QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N2, -1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N5, 1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N4, 1)), QUATER_NOTE(NOTE_TRANSHFER(N3, 1)), QUATER_NOTE(NOTE_TRANSHFER(N2, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N3, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N1, 1)), HALF_NOTE(NOTE_TRANSHFER(N5, 1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N7, 0)), HALF_NOTE(NOTE_TRANSHFER(N2, 1)), WHOLE_NOTE(NOTE_TRANSHFER(N5, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N6, 0)), HALF_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N1, -1)), WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), WHOLE_NOTE(NOTE_TRANSHFER(N3, -1)), BREAK,
    HALF_NOTE(NOTE_TRANSHFER(N5, 0)), QUATER_NOTE(NOTE_TRANSHFER(N6, 0)), QUATER_NOTE(NOTE_TRANSHFER(N1, 1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -2)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK,
    WHOLE_NOTE(NOTE_TRANSHFER(N5, 0)), HALF_NOTE(NOTE_TRANSHFER(N5, -1)), HALF_NOTE(NOTE_TRANSHFER(N2, -1)), BREAK};

volatile uint8_t TX_Message[8] = {0}; // Outgoing CAN message
volatile uint8_t RX_Message[8] = {0}; // Latest received CAN message

QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore; // Counting semaphore for CAN TX mailboxes
Osc3x osc;

#define SONG_NUM 2
Song songs[SONG_NUM]{
    Song(butterfly, sizeof(butterfly), "The Butterfly Lover"),
    Song(canon, sizeof(canon), "Canon")};

#define MAIN_KNOB_NUM 4
const uint8_t MAX_COLUMN_NUM[3]{SONG_NUM, MAIN_KNOB_NUM, SONG_NUM};
std::string knob_descript[MAIN_KNOB_NUM][4]{
    {
        "V:" // for volume
        ,
        "O:" // for octave
    },
    {"W1:",
     "A1:"},
    {"W2:",
     "A2:"},
    {"W3:",
     "A3:"}};

// Song i = Song(,"");
KeyPressBuffer keyBuffer(&osc);



// test functions
// Define a function pointer type for test functions
typedef void (*TestFuncThread)(void *);
typedef void (*TestFuncIntterupt)();

void fillmsgQ(){
  uint8_t RX_Message[8] = {'P', 0, 0, 0, 0, 0, 0, 0}; 
  for (int a = 0; a < 60; a++){
    xQueueSend(msgOutQ, RX_Message, portMAX_DELAY);
  }
}

void time_CAN_TX_Task () {
	uint8_t msgOut[8];
  for (int i=0;i<60;i++){
      xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
  }
}

// Wrapper to run a single test function 32 times and print elapsed time
void runTestThread(TestFuncThread func, const char *testName)
{
  Serial.print("Running ");
  Serial.println(testName);
  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++)
  {
    func(NULL); // Each test function is expected to run one iteration of its test.
  }
  uint32_t elapsed = micros() - startTime;
  Serial.print(testName);
  Serial.print(" elapsed time: ");
  Serial.println(elapsed/32.0 / 1000, 4); // convert to ms
}

void runTestIntterupt(TestFuncIntterupt func, const char *testName)
{
  Serial.print("Running ");
  Serial.println(testName);
  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++)
  {
    func(); // Each test function is expected to run one iteration of its test.
  }
  uint32_t elapsed = micros() - startTime;
  Serial.print(testName);
  Serial.print(" elapsed time: ");
  Serial.println(elapsed/32.0 / 1000, 4);
}

// Forward declarations of new task functions
void decodeTask(void *pvParameters)
{
  uint8_t msgIn[8];
  while (1)
  {
    #ifndef TEST_DECODE
      xQueueReceive(msgInQ, msgIn, portMAX_DELAY);
    #endif
    // Copy the received message to RX_Message.
    sysMutexAcquireRx();
    for (int i = 0; i < 8; i++)
    {
      RX_Message[i] = msgIn[i];
    }
    sysMutexReleaseRx();
    if (sysState.isReceiver.load())
    {
      if (msgIn[0] == 'P')
      {
        keyBuffer.apply_key(msgIn[2] + msgIn[3] * 12, true);
      }
      else if (msgIn[0] == 'R')
      {
        keyBuffer.apply_key(msgIn[2] + msgIn[3] * 12, false);
        // osc.release_note(msgIn[2]);
      }
      else if (msgIn[0] == 'O')
      {
        // when changing the octave
        sysState.octave.store(sysState.location + msgIn[1]);
      }
      else if (msgIn[0] == 'A')
      {
        // when there is someone checked that its neighbour left
        // sent from ends of the boards
        delay(10);
        sysState.location.store(autoDetection());
        sysState.isReceiver.store(sysState.location == 0);
      }
      else if (msgIn[0] == 'S')
      {
        sysState.song_index.store(msgIn[1]);
        songs[sysState.song_index].reset();
        sysState.mode.store(PLAYING);
      }
    }
    #ifdef TEST_DECODE
      break;
    #endif
  }
}

void CAN_TX_Task(void *pvParameters)
{
  uint8_t msgOut[8];
  while (1)
  {
#ifndef TEST_CAN_TASK
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    // Wait for a mailbox slot to be free.
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
#else
    // Test message
    msgOut[0] = 'D';
    msgOut[1] = 4;
    msgOut[2] = 5;
    msgOut[3] = 6;
#endif

    CAN_TX(0x123, msgOut);
#ifdef TEST_CAN_TASK
    break;
#endif
  }
}

// CAN receive ISR – very short, just reads message and pushes it onto msgInQ.
void CAN_RX_ISR(void)
{
  uint8_t RX_Message_ISR[8];
  uint32_t id;
  CAN_RX(id, RX_Message_ISR);
  #if defined(Disable_CAN_RegisterRX_ISR) || !defined(TEST_CAN_RX)
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  #endif
}

// CAN transmit ISR – gives the semaphore each time a mailbox slot is freed.
void CAN_TX_ISR(void)
{
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

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

// define the columns for the knobs
const uint KNOB_A[4]{18, 16, 14, 12};
const int KNOB_B[4]{19, 17, 15, 13};

const int KNOB_0_MIN = 0;
const int KNOB_0_MAX = 8;
const int KNOB_1_MIN = 0;
const int KNOB_1_MAX = 3;
const int KNOB_2_MIN = 0;
const int KNOB_2_MAX = 4;
const int KNOB_3_MIN = 0;
const int KNOB_3_MAX = 8;

// C++
// Global timer object using TIM1 from stm32duino HardwareTimer library.
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

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

class knob
{
private:
  volatile uint _prevKnob = 0;
  volatile uint _knobRotation = 0;
  volatile int _lastKnobSign = 0;
  int _minimum;
  int _maximum;
  int _A_column;

public:
  knob(int minimum, int maximum, uint A_column, int current_rotation) : _minimum(minimum), _maximum(maximum), _A_column(A_column)
  {
    setRow(3);
    delayMicroseconds(3);
    std::bitset<4> rowInputs = readCols();
    __atomic_store_n(&_knobRotation, max(min(current_rotation, _maximum), _minimum), __ATOMIC_RELAXED);
    __atomic_store_n(&_prevKnob, (rowInputs[_A_column % 4 + 1] * 2) + rowInputs[_A_column % 4], __ATOMIC_RELAXED);
  }
  int read_current()
  {
    return __atomic_load_n(&_knobRotation, __ATOMIC_RELAXED);
  }

  bool update_rotate();
};

knob knobRotation[MAIN_KNOB_NUM][4]{{knob(KNOB_0_MIN, KNOB_0_MAX, KNOB_A[0], 1),
                                     knob(KNOB_1_MIN, KNOB_1_MAX, KNOB_A[1], 1),
                                     knob(KNOB_2_MIN, KNOB_2_MAX, KNOB_A[2], 1),
                                     knob(KNOB_3_MIN, KNOB_3_MAX, KNOB_A[3], 1)},
                                    {knob(KNOB_0_MIN, 2, KNOB_A[0], 1),
                                     knob(KNOB_1_MIN, KNOB_1_MAX, KNOB_A[1], 1),
                                     knob(KNOB_2_MIN, KNOB_2_MAX, KNOB_A[2], 1),
                                     knob(KNOB_3_MIN, KNOB_3_MAX, KNOB_A[3], 1)},
                                    {knob(KNOB_0_MIN, 2, KNOB_A[0], 1),
                                     knob(KNOB_1_MIN, KNOB_1_MAX, KNOB_A[1], 1),
                                     knob(KNOB_2_MIN, KNOB_2_MAX, KNOB_A[2], 1),
                                     knob(KNOB_3_MIN, KNOB_3_MAX, KNOB_A[3], 1)},
                                    {knob(KNOB_0_MIN, 2, KNOB_A[0], 1),
                                     knob(KNOB_1_MIN, KNOB_1_MAX, KNOB_A[1], 1),
                                     knob(KNOB_2_MIN, KNOB_2_MAX, KNOB_A[2], 1),
                                     knob(KNOB_3_MIN, KNOB_3_MAX, KNOB_A[3], 1)}}; // knobs

void send_handshake_signal(int signalW, int signalE)
{
  setRow(5);
  delayMicroseconds(3);
  digitalWrite(OUT_PIN, signalW);
  delayMicroseconds(3);
  setRow(6);
  delayMicroseconds(3);
  digitalWrite(OUT_PIN, signalE);
  delayMicroseconds(3);
}

// C++
void sampleISR()
{
  // // Right-shift the waveform by (8 - volume) for simple log taper volume control
  // Vout = Vout >> (8 - volume);
  if (sysState.isReceiver.load())
  {
    auto vout = (osc.fetch_waveform_height() >> 24) - 128;
    vout = vout >> (8 - sysState.volume.load());
    analogWrite(OUTR_PIN, vout + 128);
  }
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

inline std::bitset<32> readInput()
{
  std::bitset<32> inputsTemp;
  // Scan rows 0 to 2 (12 keys total) and row3 for knob 3 rotation.
  for (uint8_t row = 0; row < 8; row++)
  {
    setRow(row);
    delayMicroseconds(3);
    std::bitset<4> rowInputs = readCols();
    for (uint8_t col = 0; col < 4; col++)
    {
      inputsTemp[row * 4 + col] = rowInputs[col];
    }
  }
  return inputsTemp;
}

void scanJoystickTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  int joystickX;
  int joystickY;
  int origin = 490;
  int deadZone = 150;
  uint8_t prev_state = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    #ifndef TEST_JOY_STICK
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif
    joystickX = analogRead(JOYX_PIN);
    joystickY = analogRead(JOYY_PIN);
    if (sysState.mode != PLAYING && sysState.mode != TEACHING)
    {
      if (abs(joystickX - origin) <= deadZone && abs(joystickY - origin) <= deadZone)
      {
        // nothing changing
        prev_state = 0;
        #ifdef TEST_JOY_STICK
      break;
    #endif
        continue;
      }
      else if (joystickX - origin > deadZone)
      {
        // if top
        if (prev_state != 1)
        {
          uint8_t page_index = (uint8_t)sysState.mode.load();
          if (page_index <= 2)
          {
            sysState.page_column_index[page_index].store((sysState.page_column_index[page_index] - 1) % MAX_COLUMN_NUM[page_index]);
            if (sysState.mode.load() != NORMAL)
            {
              sysState.song_index = sysState.page_column_index[page_index].load();
            }
          }
          prev_state = 1;
        }
      }
      else if (joystickX - origin < -deadZone)
      {
        if (prev_state != 2)
        {
          uint8_t page_index = (uint8_t)sysState.mode.load();
          if (page_index <= 2)
          {
            sysState.page_column_index[page_index].store((sysState.page_column_index[page_index] + 1) % MAX_COLUMN_NUM[page_index]);
            if (sysState.mode.load() != NORMAL)
            {
              sysState.song_index = sysState.page_column_index[page_index].load();
            }
          }
          prev_state = 2;
        }
      }
      else if (joystickY - origin > deadZone)
      {

        if (prev_state != 3)
        {
          uint8_t page_index = (uint8_t)sysState.mode;
          if (page_index <= 2)
          {
            if (page_index != 0)
            {
              sysState.mode.store((EXE_Mode)(page_index - 1));
            }
          }
          prev_state = 3;
        }
      }
      else if (joystickY - origin < -deadZone)
      {
        // if up
        // avoid of overflow

        if (prev_state != 4)
        {
          uint8_t page_index = (uint8_t)sysState.mode;
          if (page_index < 2)
          {
            sysState.mode.store((EXE_Mode)(page_index + 1));
          }
        }
        prev_state = 4;
      }
    }
    #ifdef TEST_JOY_STICK
      break;
    #endif
  }
}


// Task to scan keys and update global currentStepSize.
void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t localStepSize = 0;

  while (1)
  {
    #ifndef TEST_SCANKEYS
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif

    std::bitset<32> inputsTemp = readInput();

    // updating the global inputs
    sysMutexAcquire();
    sysState.inputs = inputsTemp;
    sysMutexRelease();

    // only update if the board is a receiver
    if (sysState.isReceiver.load())
    {
      for (int note = 0; note < 12; note++)
      {
        // Note pressed when key reads logic 0.
        if (!inputsTemp[note])
        {
          // localStepSize = stepSizes[note];
          // osc.set_note_offset(sysState.location.load() * 12);
          keyBuffer.apply_key(note + sysState.location.load() * 12, true);
        }
        else
        {
          keyBuffer.apply_key(note + sysState.location.load() * 12, false);
        }
      }
    }
    else
    {
      // Within scanKeysTask() — after scanning the key matrix.
      static std::bitset<32> prevInputs; // static variable to hold the previous key states
      for (int note = 0; note < 12; note++)
      {
        bool currPressed = !inputsTemp[note]; // pressed if logic 0
        bool prevPressed = prevInputs[note];
        if (currPressed != prevPressed)
        { // state changed
          if (currPressed)
            TX_Message[0] = 'P';
          else
            TX_Message[0] = 'R';
          TX_Message[1] = 4;                        // For example, fixed octave 4
          TX_Message[2] = note;                     // Note number
          TX_Message[3] = sysState.location.load(); // location
          // Send TX_Message via the transmit queue:
          xQueueSend(msgOutQ, (void *)TX_Message, portMAX_DELAY);
        }
        // Save the current state.
        prevInputs[note] = currPressed;
      }
    }

    static std::bitset<1> prev_joy_s;
    std::bitset<1> current_joy_s = !inputsTemp[22];
    if (prev_joy_s[0] != current_joy_s[0])
    {
      Serial.println("joy s changed");
      if (current_joy_s[0])
      {
        Serial.println("joy s pressed");
        if (sysState.mode.load() == PLAYING || sysState.mode.load() == TEACHING)
        {
          // to remove the key pressed
          for (int i = 0; i < 10; i++)
          {
            osc.articulations[i].note.store(0);
          }
          // to clean the song buffer
          for (int i = 0; i < 8; i++)
          {
            songNotes[i] = std::vector<uint8_t>();
          }
          sysState.mode.store(NORMAL);
        }
        else if (sysState.mode.load() == SELECT_PLAYING_STATE)
        {
          songs[sysState.song_index.load()].reset();
          sysState.mode.store(PLAYING);
        }
        else if (sysState.mode.load() == SELECT_TEACHING_STATE)
        {
          songs[sysState.song_index.load()].reset();
          sysState.mode.store(TEACHING);
        }
      }
      prev_joy_s[0] = current_joy_s[0];
    }
    uint8_t page_column = sysState.page_column_index[1];
    // knobRotation[page_column][0].update_rotate();
    // knobRotation[page_column][1].update_rotate();
    // knobRotation[page_column][2].update_rotate();
    if (page_column == 0)
    {
      if (knobRotation[page_column][0].update_rotate())
      {
        int i = (knobRotation[page_column][0].read_current());
        sysState.volume.store(i);
      }
      if (knobRotation[page_column][1].update_rotate())
      {
        int i = (knobRotation[page_column][1].read_current());
        osc.oscillators[0].amplitude.store(0.125f * (float)i);
      }
    }
    else
    {
      if (knobRotation[page_column][0].update_rotate())
      {
        int i = (knobRotation[page_column][0].read_current());
        osc.oscillators[page_column - 1].shape.store((WaveShape)i);
      }
      if (knobRotation[page_column][1].update_rotate())
      {
        int i = (knobRotation[page_column][1].read_current());
        osc.oscillators[page_column - 1].amplitude.store(0.125f * (float)i);
      }
    }
    #ifdef TEST_SCANKEYS
      break;
    #endif
  }
}

void drawPiano()
{

  u8g2.drawFrame(0, 28, 120, 5);

  for (int x = 15; x < 113; x += 17)
  {
    if (x != 15 + 17 * 2)
    {
      u8g2.drawBox(x, 28, 5, 3);
    }
    u8g2.drawVLine(x + 2, 28, 4);
  }
}

// for example level 0, with node 1(2 sec away) , 1 level 1/4 second
void drawNode(uint8_t node, uint8_t level, uint8_t offset)
{
  uint8_t height = 4;
  if (level == 7)
  {
    height = offset;
  }
  switch (node)
  {
  case 0:
    u8g2.drawBox(1, level * 4 - offset, 16, height);
    break;

  case 1:
    u8g2.drawBox(15, level * 4 - offset, 5, height);
    break;

  case 2:
    u8g2.drawBox(18, level * 4 - offset, 16, height);
    break;

  case 3:
    u8g2.drawBox(32, level * 4 - offset, 5, height);
    break;

  case 4:
    u8g2.drawBox(35, level * 4 - offset, 16, height);
    break;

  case 5:
    u8g2.drawBox(52, level * 4 - offset, 16, height);
    break;

  case 6:
    u8g2.drawBox(66, level * 4 - offset, 5, height);
    break;

  case 7:
    u8g2.drawBox(69, level * 4 - offset, 16, height);
    break;

  case 8:
    u8g2.drawBox(83, level * 4 - offset, 5, height);
    break;

  case 9:
    u8g2.drawBox(86, level * 4 - offset, 16, height);
    break;

  case 10:
    u8g2.drawBox(100, level * 4 - offset, 5, height);
    break;

  case 11:
    u8g2.drawBox(103, level * 4 - offset, 16, height);
    break;
  default:
    return;
    #ifdef TEST_SCANKEYS
    break;
#endif
  }
}

// Task to update the display.
void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int i = 0;
  while (1)
  {
    if (sysState.mode.load() == NORMAL && sysState.isReceiver.load())
    {
      #ifndef TEST_DISPLAY
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      #endif
      u8g2.clearBuffer();

      u8g2.setFont(u8g2_font_ncenB08_tr);
      // Display current phase step size (read atomically)
      char buf[12];
      u8g2.drawStr(2, 10, buf);

      // Also display lower 12 bits of the key matrix input.
      u8g2.setCursor(2, 20);
      sysMutexAcquire();
      std::bitset<32> inputs = sysState.inputs;
      sysMutexRelease();
      u8g2.print(inputs.to_ulong() & 0xFFF, HEX);
      u8g2.print(" ");
      u8g2.print(sysState.location.load());

      // no need for set up the mutexes
      uint8_t col_index = sysState.page_column_index[1];
      int knob = knobRotation[col_index][0].read_current();
      int knob1 = knobRotation[col_index][1].read_current();

      u8g2.setCursor(2, 30);
      u8g2.print(knob_descript[col_index][0].c_str());
      u8g2.print(knob);

      u8g2.setCursor(45, 30);
      u8g2.print(knob_descript[col_index][1].c_str());
      u8g2.print(knob1);

      // Display latest received CAN message.
      u8g2.setCursor(88, 30);
      sysMutexAcquireRx();
      // u8g2.print((char)RX_Message[0]);
      // u8g2.print(RX_Message[1]);
      // u8g2.print(RX_Message[2]);
      sysMutexReleaseRx();
      u8g2.sendBuffer();
    }
    else if (sysState.mode.load() == SELECT_PLAYING_STATE)
    {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      u8g2.clearBuffer();
      u8g2.setCursor(2, 20);
      u8g2.print(songs[sysState.song_index]._name.c_str());
      u8g2.setCursor(2, 30);
      u8g2.print("   play");
      u8g2.sendBuffer();
    }
    else if (sysState.mode.load() == SELECT_TEACHING_STATE)
    {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      u8g2.clearBuffer();
      u8g2.setCursor(2, 20);
      u8g2.print(songs[sysState.song_index]._name.c_str());
      u8g2.setCursor(2, 30);
      u8g2.print("   teach");
      u8g2.sendBuffer();
    }
    else
    {
      // change the display to about 28 hertz per second
      const TickType_t xFrequency_ = 42 / portTICK_PERIOD_MS;
      vTaskDelayUntil(&xLastWakeTime, xFrequency_);

      u8g2.clearBuffer();

      int k = songIndex;
      Serial.println("break");
      if (i % 4 == 0)
      {
        uint8_t old_index = songIndex.load();
        songIndex.store((old_index + 7) % 8);
        songNotes[songIndex] = std::move(songs[sysState.song_index.load()].get_next());
        for (uint8_t x : songNotes[old_index])
        {
          Serial.print("get: ");
          Serial.println(x);
        }

        // if it is playing update the articulations
        if (sysState.mode.load() == PLAYING)
        {
          int j = 0;
          for (uint8_t note : songNotes[(songIndex + 7) % 8])
          {
            Serial.print("printed: ");
            Serial.println(note);
            osc.articulations[j].note.store(note + 1);
            j++;
          }
          for (; j < 10; j++)
          {
            osc.articulations[j].note.store(0);
          }
        }
      }

      for (int j = 0; j < 8; j++)
      {

        for (uint8_t note : songNotes[k])
        {

          drawNode(note, j, (4 - i % 4) % 4);
        }
        // drawNode(j, j, (4 - i % 4) % 4);
        k++;
        k = k % 8;
      }

      drawPiano();
      i++;
      i = i % 28;
      u8g2.sendBuffer();
    }

    digitalToggle(LED_BUILTIN); // Blink LED to indicate timing
#ifdef TEST_DISPLAY
    break;
#endif
  }
}

uint8_t autoDetection()
{
  uint8_t msgOut[8] = {0}; // message out
  uint8_t msgIn[8] = {0};  // message in

  std::bitset<32> inputs;
  std::bitset<1> WestDetect;
  std::bitset<1> EastDetect;

  for (int i = 0; i < 10; i++)
  {
    send_handshake_signal(1, 1);
    delayMicroseconds(30000);
  }

  inputs = readInput();
  WestDetect[0] = inputs[23];
  EastDetect[0] = inputs[27];

  sysState.east_record.store(EastDetect[0]);
  sysState.west_record.store(WestDetect[0]);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(20, 20);
  u8g2.print("loading");
  u8g2.sendBuffer();

  if (WestDetect[0])
  {
    Serial.println("mainboard");

    if (!EastDetect[0])
    {
      Serial.println("it is not the only board.");

      msgOut[0] = 'C';
      msgOut[1] = 0; // For example, fixed octave 4
      // Send TX_Message via the transmit queue:
      CAN_TX(0x123, msgOut);
      Serial.println("submitted C 0 in CAN");
      // delay(200);
      delay(300);
      for (int i = 0; i < 10; i++)
      {
        send_handshake_signal(0, 0);
        delay(30);
      }
      Serial.println("disabled the east west");
      delay(300);
    }
    else
    {
      return 0;
    }
    while (true)
    {
      Serial.println("waiting for finishing");
      uint8_t msgIn[8];
      uint32_t id;
      CAN_RX(id, msgIn);
      if (msgIn[0] == 'D')
      {
        break;
      };
      delayMicroseconds(30000);
    }
    return 0;
  }
  else
  {
    do
    {
      inputs = readInput();
      WestDetect[0] = inputs[23];
      delay(30);
      Serial.println("detecting");
    } while (!WestDetect[0]);
    Serial.println("non detecting west");
    uint8_t msgIn[8];
    int id = 0;
    delay(20);
    Serial.println(CAN_CheckRXLevel());
    while (CAN_CheckRXLevel() > 0)
    {
      uint32_t ID;
      CAN_RX(ID, msgIn);
      if (msgIn[0] == 'C')
      {
        if (id < msgIn[1])
        {
          Serial.println("received C");
          Serial.println(msgIn[1]);
          id = msgIn[1];
        }
      }
    }
    uint8_t msgOut[8];
    msgOut[0] = 'C';
    msgOut[1] = id + 1; // For example, fixed octave 4
    // Send TX_Message via the transmit queue:
    CAN_TX(0x123, msgOut);
    if (EastDetect[0])
    {
      msgOut[0] = 'D';
      CAN_TX(0x123, msgOut);
      for (int i = 0; i < 10; i++)
      {
        send_handshake_signal(0, 0);
        delay(30);
      }
      Serial.println("finished");
      return id + 1;
    }
    for (int i = 0; i < 10; i++)
    {
      send_handshake_signal(0, 0);
      delay(30);
    }
    while (true)
    {
      uint8_t msgIn[8];
      uint32_t id;
      CAN_RX(id, msgIn);
      delayMicroseconds(30000);
      if (msgIn[0] == 'D')
      {
        break;
      };
    }
    return id + 1;
  }


}

void loop()
{
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

  // --- CAN bus initialisation ---
  CAN_Init(true);            // Loopback mode for testing
  setCANFilter(0x123, 0x7FF); // Only accept messages with ID 0x123
  osc.oscillators[0].amplitude = 0.125f;
  osc.oscillators[1].amplitude = 0.125f;
  osc.oscillators[2].amplitude = 0.125f;
  // Create counting semaphore for 3 mailbox slots.
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  // Register CAN TX and RX ISRs.
  #ifndef Disable_CAN_RegisterRX_ISR
    CAN_RegisterRX_ISR(CAN_RX_ISR);
  #endif
  #ifndef Disable_CAN_RegisterTX_ISR
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();
  // Initialise Queues
  #ifndef Disable_msgque
    msgInQ = xQueueCreate(36, 8);
    msgOutQ = xQueueCreate(36, 8);
  #else
    msgInQ = xQueueCreate(384, 8);
    msgOutQ = xQueueCreate(384, 8);
  #endif
  

  sysState.location.store(autoDetection());
  send_handshake_signal(1, 1);
  if (sysState.location == 0)
  {
    sysState.isReceiver.store(true);
  }

  Serial.println("LOCATION:");
  Serial.println(sysState.location);
  // Configure timer to trigger the sampleISR() at 22kHz.
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  #ifndef Disable_attachInterrupt_sampleISR
    sampleTimer.attachInterrupt(sampleISR);
  #endif
  sampleTimer.resume();

  // Create FreeRTOS tasks.
#ifndef DISABLE_THREADS

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      1,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  // // TaskHandle_t displayHandle = NULL, 1, &displayHandle);
  TaskHandle_t displayHandle = NULL;
  xTaskCreate(displayUpdateTask, "display", 256, NULL, 1, &displayHandle);

  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(decodeTask, "decode", 128, NULL, 1, &decodeHandle);

  TaskHandle_t canTxHandle = NULL;
  xTaskCreate(CAN_TX_Task, "CAN_TX", 128, NULL, 1, &canTxHandle);
  
  TaskHandle_t joyHandle = NULL;
  xTaskCreate(scanJoystickTask, "scan_joy_input", 128, NULL, 1, &joyHandle);
#endif

  // Create a mutex to protect the shared sysState.inputs.
  sysState.mutex = xSemaphoreCreateMutex();

  // Create a mutex to protect the shared sysState.rx_meg_mutex.
  sysState.rx_meg_mutex = xSemaphoreCreateMutex();

// osc.oscillators[1].amplitude.store(0.2f);
// osc.oscillators[1].shape.store(TriangularWave);
// osc.oscillators[2].amplitude.store(0.2f);
// osc.oscillators[2].shape.store(SawtoothWave);

// testing thread
#ifdef TEST_SCANKEYS
  runTestThread(scanKeysTask, "scanKeysTask");
#endif
#ifdef TEST_DISPLAY
  runTestThread(displayUpdateTask, "displayUpdateTask");
#endif
#ifdef TEST_DECODE
  runTestThread(decodeTask, "decodeTask");
#endif
#ifdef TEST_SAMPLEISR
  runTestIntterupt(sampleISR, "sampleISR");
#endif
#ifdef TEST_JOY_STICK
  runTestThread(scanJoystickTask, "scanJoystickTask");
#endif
#ifdef TEST_CAN_TASK
  runTestThread(CAN_TX_Task, "CAN_Task");
#endif
#ifdef TEST_CAN_TX
  runTestIntterupt(CAN_TX_ISR, "CAN_TX_ISR");
#endif

#ifdef TEST_CAN_RX // run with Disable_CAN_RegisterRX_ISR TEST_CAN_TASK
  Serial.print("Running ");
  uint32_t elapsed_tx = 0;
  uint32_t startTime = 0;
  CAN_TX_Task(NULL);
  startTime = micros();
  CAN_RX_ISR();
  elapsed_tx += micros() - startTime;
  Serial.print("CAN_RX_ISR elapsed time in millisec: ");
  Serial.println(elapsed_tx, 4);
#endif

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();
}

bool knob::update_rotate()
{
  {
    sysMutexAcquire();
    std::bitset<32> input = sysState.inputs;
    sysMutexRelease();

    // _B_column will always = _A_column + 1
    uint8_t currentKnob = (input[_A_column + 1] * 2) + input[_A_column];
    // in case of possible changing the prevKnob on the way
    int prevKnob = __atomic_load_n(&_prevKnob, __ATOMIC_RELEASE);
    __atomic_store_n(&_prevKnob, currentKnob, __ATOMIC_ACQUIRE);

    int delta = 0;
    // Only update rotation when A (bit0) changes.
    if (prevKnob == 0 && currentKnob == 1 || prevKnob == 3 && currentKnob == 2)
      delta = 1;
    else if (prevKnob == 1 && currentKnob == 0 || prevKnob == 2 && currentKnob == 3)
      delta = -1;
    // If both bits changed (impossible transition), assume same direction as last valid.
    if (prevKnob == 0 && currentKnob == 3 || prevKnob == 3 && currentKnob == 0 || prevKnob == 1 && currentKnob == 2 || prevKnob == 2 && currentKnob == 1)
      delta = __atomic_load_n(&_lastKnobSign, __ATOMIC_RELEASE);

    __atomic_store_n(&_lastKnobSign, delta, __ATOMIC_ACQUIRE);

    // Update knob rotation using atomic store.
    int tempRotation = __atomic_load_n(&_knobRotation, __ATOMIC_RELAXED);
    int newRotation = tempRotation + delta;
    newRotation = max(min(newRotation, _maximum), _minimum);
    __atomic_store_n(&_knobRotation, newRotation, __ATOMIC_RELAXED);
    return delta != 0;
  }
}
