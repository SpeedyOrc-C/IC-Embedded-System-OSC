#include <Arduino.h>
#include <U8g2lib.h>

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

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

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
}

enum KeyboardKey
{
    KeyC = 0,
    KeyCs = 1,
    KeyDf = 1,
    KeyD = 2,
    KeyDs = 3,
    KeyEf = 3,
    KeyE = 4,
    KeyF = 5,
    KeyFs = 6,
    KeyGf = 6,
    KeyG = 7,
    KeyGs = 8,
    KeyAf = 8,
    KeyA = 9,
    KeyAs = 10,
    KeyBf = 10,
    KeyB = 11,
};

bool TestKey(KeyboardKey key)
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

    switch (key)
    {
    case KeyC:
    case KeyE:
    case KeyGs:
        return digitalRead(C0_PIN);

    }
}

KeyboardKey NextKeyOf(KeyboardKey key)
{
    if (key == KeyB)
        return KeyC;

    return (KeyboardKey)(((int)key) + 1);
}

void loop()
{
    static uint32_t next = millis();
    static uint32_t count = 0;

    while (millis() < next)
        ;

    next += interval;

    u8g2.clearBuffer();
    for (int key = 0; key <= 11; key += 1)
    {
        const bool isDown = TestKey((KeyboardKey)key);
        const int x = key;

        if (isDown)
        {
            u8g2.drawBox(x * 8, 0, 8, 8);
        }
    }
    u8g2.sendBuffer();
}
