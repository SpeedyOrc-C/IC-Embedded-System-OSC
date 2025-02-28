#include <Osc3x.h>

#define MAX_KEYBOARD_KEY_COUNT 12

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

class KeyPressBuffer
{
public:
    bool keys[MAX_KEYBOARD_KEY_COUNT] = {false, false, false, false, false, false, false, false, false, false, false, false};
    Osc3x *synthesizer;

    KeyPressBuffer(Osc3x *synthesizer)
    {
        this->synthesizer = synthesizer;
    }

    void apply_key(KeyboardKey key, bool is_pressed)
    {
        if (keys[key] != is_pressed)
        {
            keys[key] = is_pressed;

            if (is_pressed)
                synthesizer->press_note(key);
            else
                synthesizer->release_note(key);
        }
    }
};
