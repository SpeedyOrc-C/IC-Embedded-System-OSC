#include <stdint.h>
#include <vector>
#include <string>
#include <atomic>


#define NOTE_TRANSHFER(x, y) (x + (y + 2)*12)
#define BREAK 255
#define WHOLE_STOP 254
#define HALF_STOP 253
#define QUATER_STOP 252
#define WHOLE_NOTE(x) x, 4
#define HALF_NOTE(x) x, 2
#define QUATER_NOTE(x) x, 1
#define N1 0
#define N1_ 1
#define N2 2
#define N2_ 3
#define N3 4
#define N4 5
#define N4_ 6
#define N5 7
#define N5_ 8
#define N6 9
#define N6_ 10
#define N7 11


class Song{
private:
    int _index = 0;
    const uint8_t* _notes;
    size_t _size = 0;
    std::atomic<uint8_t> _offset = 0;
public:
    std::string _name;
    Song() = delete;
    Song(const uint8_t *notes , size_t notes_size, std::string &&name){
        _notes = notes;
        _name = std::move(name);
        _size = notes_size;
    };
    std::vector<uint8_t> get_next(){
        Serial.println("GO_NEXT");
        std::vector<uint8_t> answer;
        int i = _index;
        int note_count = 0;
        if(i >= _size) {
            return std::move(answer);
        }
        while(_notes[i] != BREAK){
            // Serial.print("i: ");
            // Serial.println(i);
            note_count = note_count % 4;
            if(i == _size){
                Serial.println("out of bound, music fault");
                return std::move(answer);
            }
            if(_notes[i] == WHOLE_STOP) {
                i++;
                continue;
            } else if(_notes[i] == HALF_STOP) {
                i++;
                note_count += 2;
                continue;
            } else if(_notes[i] == QUATER_STOP) {
                i++;
                note_count += 2;
                continue;
            }
            
            if(i + 1 == _size){
                Serial.println("out of bound, music fault");
                return std::move(answer);
            }

            // Serial.print("offset: ");
            // Serial.println(_offset);
            // Serial.print("note_count: ");
            // Serial.println(note_count);
            if(note_count <= _offset && note_count + _notes[i + 1] - 1 >= _offset){
                answer.emplace_back(_notes[i]);
                Serial.print("out: ");
                Serial.println(_notes[i]);
            }
            note_count += _notes[i + 1];
            // Serial.print("_notes[i + i] ");
            // Serial.println(note_count);
            // Serial.print("_notes[i + i] ");
            // Serial.println(note_count);
            // Serial.print("note_count_aft: ");
            // Serial.println(note_count);
            i += 2;
        }

        _offset += 1;
        if(_offset >= 4) {
            _offset = _offset % 4;
            _index = i + 1;
        }
        
        return std::move(answer);
    }
    void reset(){
        _index = 0;
    }

};