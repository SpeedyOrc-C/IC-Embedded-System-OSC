#include <atomic>
#include <Osc3x.h>
#include <STM32FreeRTOS.h>

#define MAX_KEYBOARD_KEY_COUNT 108

// enum KeyboardKey
// {
//     KeyC = 0,
//     KeyCs = 1,
//     KeyDf = 1,
//     KeyD = 2,
//     KeyDs = 3,
//     KeyEf = 3,
//     KeyE = 4,
//     KeyF = 5,
//     KeyFs = 6,
//     KeyGf = 6,
//     KeyG = 7,
//     KeyGs = 8,
//     KeyAf = 8,
//     KeyA = 9,
//     KeyAs = 10,
//     KeyBf = 10,
//     KeyB = 11,
// };

struct KnobReading
{
public:
    bool a1 : 1;
    bool b1 : 1;
    bool a2 : 1;
    bool b2 : 1;
    bool a3 : 1;
    bool b3 : 1;
    bool a4 : 1;
    bool b4 : 1;
};

class KeyPressBuffer
{
public:
    std::atomic<bool> key_reading[MAX_KEYBOARD_KEY_COUNT] = {false, false, false, false, false, false, false, false, false, false, false, false};
    Osc3x *synthesizer;

    KeyPressBuffer(Osc3x *synthesizer)
    {
        this->synthesizer = synthesizer;
    }

    void apply_key(int key, bool is_pressed)
    {
        bool not_is_pressed = !is_pressed;
        if (key_reading[key].compare_exchange_strong(not_is_pressed, is_pressed))
        {
            if (is_pressed)
                synthesizer->press_note(key);
            else
                synthesizer->release_note(key);
        }
    }
};
