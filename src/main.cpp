#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <atomic>
#include <KeyPressBuffer.h>
#include <Osc3x.h>

// Constants
const uint32_t interval = 200; // Display update interval

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

Osc3x osc;
KeyPressBuffer key_press_buffer(&osc);

HardwareTimer sample_timer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

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

bool test_key_up(KeyboardKey key)
{
    digitalWrite(RA2_PIN, LOW);

    switch (key)
    {
    case KeyC:
    case KeyCs:
    case KeyD:
    case KeyDs:
        digitalWrite(RA0_PIN, LOW);
        digitalWrite(RA1_PIN, LOW);
        break;
    case KeyE:
    case KeyF:
    case KeyFs:
    case KeyG:
        digitalWrite(RA0_PIN, HIGH);
        digitalWrite(RA1_PIN, LOW);
        break;
    case KeyGs:
    case KeyA:
    case KeyAs:
    case KeyB:
        digitalWrite(RA0_PIN, LOW);
        digitalWrite(RA1_PIN, HIGH);
        break;
    }

    digitalWrite(REN_PIN, HIGH);
    delayMicroseconds(2);

    bool value;

    switch (key)
    {
    case KeyC:
    case KeyE:
    case KeyGs:
        value = digitalRead(C0_PIN);
        break;
    case KeyCs:
    case KeyF:
    case KeyA:
        value = digitalRead(C1_PIN);
        break;
    case KeyD:
    case KeyFs:
    case KeyAs:
        value = digitalRead(C2_PIN);
        break;
    case KeyDs:
    case KeyG:
    case KeyB:
        value = digitalRead(C3_PIN);
        break;
    }

    digitalWrite(REN_PIN, LOW);

    return value;
}

KnobReading test_knobs()
{
    KnobReading reading;

    digitalWrite(REN_PIN, HIGH);

    digitalWrite(RA2_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA0_PIN, HIGH);

    delayMicroseconds(2);

    reading.a4 = digitalRead(C0_PIN);
    reading.b4 = digitalRead(C1_PIN);
    reading.a3 = digitalRead(C2_PIN);
    reading.b3 = digitalRead(C3_PIN);

    digitalWrite(RA2_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA0_PIN, LOW);

    delayMicroseconds(2);

    reading.a2 = digitalRead(C0_PIN);
    reading.b2 = digitalRead(C1_PIN);
    reading.a1 = digitalRead(C2_PIN);
    reading.b1 = digitalRead(C3_PIN);

    digitalWrite(REN_PIN, LOW);

    return reading;
}

void send_waveform_height()
{
    const uint32_t height = osc.fetch_waveform_height();
    const uint32_t height8 = height >> 24;

    analogWrite(OUTR_PIN, height8);
}

void setup()
{
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

    u8g2.setFont(u8g2_font_ncenB08_tr);

    sample_timer.setOverflow(22000, HERTZ_FORMAT);
    sample_timer.attachInterrupt(send_waveform_height);
    sample_timer.resume();
}

void loop()
{
    for (int key = 0; key < MAX_KEYBOARD_KEY_COUNT; key += 1)
        key_press_buffer.apply_key((KeyboardKey)key, !test_key_up((KeyboardKey)key));

    key_press_buffer.apply_knob(test_knobs());

    u8g2.clearBuffer();

    for (int key = 0; key <= 11; key += 1)
        if (key_press_buffer.key_reading[key])
            u8g2.drawBox(key * 4, 12, 4, 4);

    for (int i = 0; i < 10; i += 1)
        if (osc.articulations[i].activated.load())
            u8g2.drawBox(i * 4, 17, 4, 4);

    u8g2.setCursor(2, 10);
    u8g2.print(osc.tick.load());

    u8g2.sendBuffer();
}
