#ifndef H_OSC3X
#define H_OSC3X

#include <cstdint>
#include <mutex>
#include <atomic>
#include <cmath>

#define MAX_OSCILLATOR_COUNT 3
#define MAX_ARTICULATION_COUNT 10

const float memo_2_pow_x_div_12[37] = {
    1.0f,
    1.0594630943592953f,
    1.122462048309373f,
    1.189207115002721f,
    1.2599210498948732f,
    1.3348398541700344f,
    1.4142135623730951f,
    1.4983070768766815f,
    1.5874010519681994f,
    1.681792830507429f,
    1.7817974362806785f,
    1.8877486253633868f,
    2.0f,
    2.1189261887185906f,
    2.244924096618746f,
    2.378414230005442f,
    2.5198420997897464f,
    2.6696797083400687f,
    2.8284271247461903f,
    2.996614153753363f,
    3.174802103936399f,
    3.363585661014858f,
    3.563594872561357f,
    3.775497250726774f,
    4.0f,
    4.237852377437181f,
    4.489848193237491f,
    4.756828460010884f,
    5.039684199579493f,
    5.339359416680137f,
    5.656854249492381f,
    5.993228307506727f,
    6.3496042078727974f,
    6.727171322029716f,
    7.127189745122715f,
    7.550994501453547f,
    8.0f,
};

const float memo_2_pow_neg_x_div_12[37] = {
    1.0f,
    0.9438743126816935f,
    0.8908987181403393f,
    0.8408964152537145f,
    0.7937005259840998f,
    0.7491535384383408f,
    0.7071067811865476f,
    0.6674199270850172f,
    0.6299605249474366f,
    0.5946035575013605f,
    0.5612310241546865f,
    0.5297315471796477f,
    0.5f,
    0.47193715634084676f,
    0.44544935907016964f,
    0.42044820762685725f,
    0.3968502629920499f,
    0.3745767692191704f,
    0.3535533905932738f,
    0.3337099635425086f,
    0.3149802624737183f,
    0.29730177875068026f,
    0.28061551207734325f,
    0.2648657735898238f,
    0.25f,
    0.23596857817042335f,
    0.22272467953508485f,
    0.21022410381342863f,
    0.19842513149602492f,
    0.18728838460958522f,
    0.1767766952966369f,
    0.16685498177125427f,
    0.15749013123685915f,
    0.14865088937534013f,
    0.1403077560386716f,
    0.1324328867949119f,
    0.125f,
};

enum WaveShape
{
    SquareWave,
    TriangularWave,
    SawtoothWave,
    NoiseWave,
};

struct Oscillator
{
public:
    std::atomic<WaveShape> shape = TriangularWave;
    std::atomic<float> amplitude = 0.0f;
    std::atomic<int32_t> note_offset = 0;
    std::atomic<float> phase_offset = 0.0f;
};

struct Articulation
{
public:
    std::atomic<int8_t> note = 0;
    std::atomic<bool> activated = false;
    std::atomic<int32_t> last_activation_tick = 0;
};

class Osc3x
{
public:
    const int32_t sample_rate = 22000;
    std::atomic<int32_t> tick = 0;
    std::atomic<int32_t> sustain_tick = 0;

    Oscillator oscillators[MAX_OSCILLATOR_COUNT];
    Articulation articulations[MAX_ARTICULATION_COUNT];

    uint32_t fetch_waveform_height()
    {
        float total_height = 0.0f;

        int32_t sound_count = 0;

        for (auto &oscillator : oscillators)
        {
            if (oscillator.amplitude.load() == 0.0f)
                continue;

            float oscillator_height = 0;

            for (auto &articulation : articulations)
            {
                const int32_t dt = tick - articulation.last_activation_tick.load();

                if (sustain_tick > 0 && dt > sustain_tick)
                {
                    articulation.activated.store(false);
                    continue;
                }

                if (!articulation.activated.load())
                    continue;

                sound_count += 1;

                const int32_t total_offset = articulation.note.load() + oscillator.note_offset - 9;
                const float frequency = 440.0f * (total_offset >= 0 ? memo_2_pow_x_div_12[total_offset]
                                                                    : memo_2_pow_neg_x_div_12[-total_offset]);
                const float phase = (float)tick.load() * frequency / (float)sample_rate + oscillator.phase_offset;
                float phase_int_part;
                const float phase_decimal_part = modff(phase, &phase_int_part);

                float height;

                switch (oscillator.shape)
                {
                case SquareWave:
                    height = phase_decimal_part < 0.5f ? 1.0f : -1.0f;
                    break;

                case TriangularWave:
                    if (phase_decimal_part < 0.25f)
                        height = 4.0f * phase_decimal_part;
                    else if (phase_decimal_part < 0.75f)
                        height = -4.0f * (phase_decimal_part - 0.5f);
                    else
                        height = -4.0f * (1.0f - phase_decimal_part);
                    break;

                case SawtoothWave:
                    height = 2.0f * (phase_decimal_part - 0.5f);
                    break;

                case NoiseWave:
                    height = (float)random() * 2 / (float)UINT32_MAX - 1.0f;
                    break;

                default:
                    height = 0.0f;
                    break;
                }

                if (sustain_tick > 0)
                    height *= 1.0f - (float)dt / (float)sustain_tick;

                oscillator_height += height;
            }

            total_height += oscillator_height * oscillator.amplitude;
        }

        if (sound_count > 0)
        {
            tick.fetch_add(1);
        }
        else
        {
            tick.store(0);
        }

        if (total_height > 1.0f)
            total_height = 1.0f;

        if (total_height < -1.0f)
            total_height = -1.0f;

        return (uint32_t)(((total_height + 1.0f) / 2.0f) * UINT32_MAX);
    }

    bool press_note(int8_t note)
    {
        if (sustain_tick > 0)
        {
            Articulation *free_articulation = nullptr;

            for (auto &articulation : articulations)
            {
                if (!articulation.activated.load())
                {
                    if (free_articulation == nullptr)
                        free_articulation = &articulation;
                }
                else if (articulation.note == note)
                {
                    articulation.last_activation_tick.store(tick);
                    return true;
                }
            }

            if (free_articulation == nullptr)
                return false;

            free_articulation->note.store(note);
            free_articulation->activated.store(true);
            free_articulation->last_activation_tick.store(tick);

            return true;
        }
        else
        {
            for (auto &articulation : articulations)
            {
                if (articulation.activated.load())
                    continue;

                articulation.note.store(note);
                articulation.activated.store(true);
                articulation.last_activation_tick.store(tick);

                return true;
            }

            return false;
        }
    }

    bool release_note(int8_t note)
    {
        for (auto &articulation : articulations)
        {
            if (articulation.note.load() != note)
            {
                continue;
            }

            if (sustain_tick == 0)
                articulation.activated.store(false);

            return true;
        }

        return false;
    }

    void set_note_offset(int32_t note_offset)
    {
        for (auto &oscillator : oscillators)
        {
            oscillator.note_offset.store(note_offset);
        }
    }

    void set_sustain_tick(float sustain_sec)
    {
        sustain_tick.store((int32_t)(sustain_sec * sample_rate));
    }

    Osc3x()
    {
        oscillators[0].amplitude.store(0.2f);
    }
};

#endif
