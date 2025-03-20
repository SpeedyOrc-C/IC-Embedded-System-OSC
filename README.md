# Real-Time Systems Analysis Report for StackSynth

This report analyses the real-time tasks, scheduling and data sharing mechanisms implemented in the StackSynth synthesizer project.

## Task Identification and Implementation

### Audio Processing (Interrupt-based)

- **sampleISR**  
  - **Type:** Interrupt Service Routine (ISR)
  - **Functionality:** Handles digital audio synthesis at the hardware level. The ISR's primary function is to:
    - Update a 32-bit phase accumulator based on the current frequency step size
    - Convert the phase value to an output voltage level (using bit shifting)
    - Apply volume control through a logarithmic amplitude scaling
    - Write the resulting value to the DAC via analogWrite()
    - When connecting with keyboard, this will only works if the keyboard is act as receiver
  - **Critical Aspects:** Must complete within microseconds to maintain audio quality; directly affects sound generation

### CAN Bus Communication (Interrupt-based)

- **CAN_RX_ISR**
  - **Type:** ISR
  - **Functionality:** Manages incoming CAN messages by:
    - Reading frame data from CAN hardware when a message arrives
    - Copying the 8-byte message into a temporary buffer
    - Pushing the message onto the msgInQ queue for later processing
    - Never performs blocking operations or computationally intensive work
  - **Critical Aspects:** Ensures no messages are missed; CAN hardware can only buffer 3 messages

- **CAN_TX_ISR**
  - **Type:** ISR
  - **Functionality:** Manages CAN transmit mailbox availability by:
    - Being triggered whenever a CAN transmission completes
    - Giving a semaphore to signal that a mailbox slot has become available
    - Allowing the CAN_TX_Task to send another message
  - **Critical Aspects:** Maintains CAN transmission flow; prevents transmit task blocking

### Input Scanning and Processing (Thread-based)

- **scanKeysTask**
  - **Type:** FreeRTOS Task (Thread)
  - **Functionality:** Provides keyboard input scanning and processing by:
    - we create a helper function readInput() for
        - Sequentially activating each row of the key matrix (via setRow())
        - Reading each column's state to detect key presses/releases
        - Detecting changes in key states between consecutive scans
        - return the key states at last
    - add key into keybuffer for pressing if any of the key is pressed via iteration
    - Creating CAN messages when keys change state (pressed/released)
    - Decoding rotary encoder knob movements using quadrature signals
    - Updating system parameters (e.g., volume control for knob 3, octave for knob 2, wave for knob 1) based on knob rotation
    - Scanning at a fast rate (20ms) to catch brief input changes
  - **Critical Aspects:** Must run frequently enough to catch key and knob transitions

### User Interface and Feedback (Thread-based)

- **displayUpdateTask** TODO
  - **Type:** Thread
  - **Functionality:** Updates the visual feedback systems by:
    - Reading current system parameters (frequency, key states, etc.)
    - Formatting data for user-friendly display (hex formats, text, etc.)
    - Writing to the OLED display buffer
    - Toggling LED as a timing check indicator (provides visual confirmation that system timing requirements are being met)
    - Running at a human-readable update rate (100ms)
  - **Critical Aspects:** Provides visual feedback without interfering with audio operations

### Message Handling (Thread-based)

- **decodeTask**
  - **Type:** Thread
  - **Functionality:** Processes received CAN messages by:
    - Blocking on the message input queue until messages arrive
    - Extracting command/data from CAN message structure
    - Interpreting 'P'(press) and 'R'(release) commands with note information
    - Applying key press/release to the appropriate oscillator via the KeyPressBuffer if current keyboard is receiver
  - **Critical Aspects:** Controls polyphony and distributed keyboard functionality
- **CAN_TX_Task**
  - **Type:** Thread
  - **Functionality:** Manages outgoing CAN message transmission by:
    - Waiting for messages to appear in the outgoing message queue
    - Acquiring a CAN transmit mailbox via semaphore
    - Sending the message data to the CAN controller
    - Managing message priority and timing
  - **Critical Aspects:** Ensures reliable message transmission without blocking upstream tasks

## Task Characterisation

### Execution Time and Initiation Interval Analysis

| Task              | Initiation Interval                 | Measured Execution Time | Notes                                                        |
| ----------------- | ----------------------------------- | ----------------------- | ------------------------------------------------------------ |
| sampleISR         | 45 μs (22 kHz)                      | 16 μs                   | Fixed hardware timer interval, no optional execution  paths  |
| CAN_RX_ISR        | Event-driven                        | 3-5 μs                  | Minimal processing, only queue insertion                     |
| CAN_TX_ISR        | Event-driven                        | 1-2 μs                  | Simple semaphore operation                                   |
| scanKeysTask      | 20 ms                               | 150 μs                  | Scales with key matrix size; worst case when all keys  change state |
| displayUpdateTask | 100 ms                              | 15ms                    | Display writes dominate execution time                       |
| decodeTask        | Event-driven (~25 ms worst-case)    | 4 μs                    | Processing time depends on message type                      |
| CAN_TX_Task       | Event-driven (~1.67 ms per message) | 869 μs                  | Fast operation, mostly waits on hardware                     |

## Critical Instant Analysis

Using rate-monotonic scheduling theory and the measured execution times:

1. **Periodic Tasks (ordered by priority):**
   - sampleISR: 16μs / 45μs = 0.356 utilization
   - scanKeysTask: 150μs / 20ms = 0.008 utilization
   - displayUpdateTask: 15ms / 100ms = 0.15 utilization
2. **Event-driven Tasks:**
   - CAN_RX_ISR: Negligible overhead, only active when messages arrive
   - CAN_TX_ISR: Negligible overhead, only active after transmission
   - decodeTask: Bounded by message arrival rate
   - CAN_TX_Task: Bounded by message generation rate
3. **Schedulability Test:**
   - Sum of utilizations = 0.356 + 0.008 + 0.15 = 0.514

For three periodic tasks, the theoretical rate-monotonic bound is approximately 77.9% (for 3 tasks), so 51.4% < 77.9%.

**Conclusion:** The tasks are schedulable under worst-case conditions.

## CPU Utilisation

| Task              | Individual Utilisation | Calculation                                          |
| ----------------- | ---------------------- | ---------------------------------------------------- |
| sampleISR         | 35.6%                  | 10μs × 22,000 Hz                                     |
| scanKeysTask      | 0.6%                   | 100μs × 50 Hz                                        |
| displayUpdateTask | 15%                    | 3ms × 10 Hz                                          |
| CAN_RX_ISR        | 0.05% (estimated)      | 5μs per message, ~100 messages/sec worst case        |
| CAN_TX_ISR        | 0.02% (estimated)      | 2μs per message, ~100 messages/sec worst case        |
| decodeTask        | 0.01% (estimated)      | 1ms × 10 Hz (average rate of processing)             |
| CAN_TX_Task       | 0.87% (estimated)      | 0.5ms × 10 Hz (average rate of processing)           |
| **Total**         | **~51.4%**             | Significant headroom remains for additional features |

## Shared Data Structures and Synchronisation

### Primary Shared Resources

| Resource            | Data Type / Location                                         | Protection Method                                            |
| ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| sysState.inputs     | std::bitset<32> (declared in main.cpp)                       | Protected by a FreeRTOS mutex (sysState.mutex) to  guarantee exclusive access during multi-step read/write operations. |
| TX_Message          | volatile uint8_t[8] (initially global in main.cpp,  later localised in scanKeysTask) | Local copies are used in tasks to minimize blocking;  any global access (e.g., for display updates) is guarded by a mutex if  necessary. |
| RX_Message          | volatile uint8_t[8] (global variable in  KeyPressBuffer.h)   | A dedicated mutex (e.g., rx_msg_mutex) protects  RX_Message when updated by the decode task and read by the display update  task. |
| msgInQ              | FreeRTOS Queue                                               | The queue is created with thread-safe FreeRTOS  primitives, ensuring that incoming CAN messages are safely buffered between  the ISR and the decoder task. |
| msgOutQ             | FreeRTOS Queue                                               | Similarly, the outgoing message queue provides a  thread-safe mechanism for handling CAN transmit requests from multiple  sources. |
| sysstate.isReceiver | std::atomic\<bool\> (declared in main.cpp)                   | Atomic                                                       |
| sysState.location   | std::atomic<uint8_t>(declared in main.cpp)                   | Atomic                                                       |
| keyBuffer           | KeyPressBuffer (class)                                       | ensure synchornisation in method                             |
| osc                 | Osc3x (class)                                                | ensure synchornisation in method                             |

### Synchronisation Strategy:

- **Atomic Operations**: Used for single-word variables accessed in both task and ISR contexts
- **Mutexes**: Used for protecting multi-word data structures accessed only by tasks
- **Queues**: Used for passing messages between components

## helper function

### Knob Class

The Knob class is responsible for decoding rotary encoder (knob) inputs. It tracks the current rotation and ensures that the rotation value stays within specified limits (for example, a volume knob from 0 to 8).

### Osc3x Class

The Osc3x class manages the audio synthesis functionality. It updates oscillator parameters such as the phase accumulator and step size, and it applies volume control to the synthesized audio signal.

- **Oscillators Array:**

 The class contains an array of **Oscillator** structures.

​	Each oscillator holds parameters such as:

​	**Waveform shape** (Square, Triangular, Sawtooth)

​	**Amplitude**, which determines the oscillator’s contribution to the final waveform.

​	**Note offset** and **phase offset**, which allow fine tuning and alignment of the oscillator relative to the note being played.

- **Waveform Computation – fetch_waveform_height():**

​	This method is the heart of the audio synthesis process. It computes the overall waveform amplitude by iterating over all oscillators and their associated articulations.

​	For each active oscillator and articulation:

​	It calculates the effective frequency based on the note, oscillator’s note offset, and precomputed pitch scaling factors (using lookup tables such as memo_2_pow_x_div_12).

​	It computes the phase for the oscillator using the global tick and the oscillator’s phase offset.

​	Based on the selected waveform shape, it generates a waveform value (for example, a square wave toggles between -1.0 and 1.0, while a sawtooth or triangle wave produces a linear ramp).

​	The contributions of each oscillator are summed, scaled by the oscillator’s amplitude, and clamped to a range of -1.0 to 1.0.

​	Finally, the result is converted into a 32-bit unsigned integer suitable for driving the DAC or PWM output.

- **Note Management:** **press_note(int32_t note)** and **release_note(int32_t note)**

These methods manage the activation and deactivation of notes (articulations). When a key is pressed, press_note() finds an available articulation slot and stores the note (offset by one, so 0 indicates inactive). Conversely, release_note() clears the corresponding articulation.

- **set_note_offset(int32_t note_offset)**

This method sets the note offset for all oscillators, which adjusts their pitch relative to the incoming note data.

### KeyPressBuffer Class

The KeyPressBuffer class encapsulates the storage and formatting of key press and key release messages. These messages are used to notify other modules when keys are pressed or released.

### send_handshake_signal()

The send_handshake_signal() function is used during module auto-detection and system initialization. It sends a predefined handshake message over the CAN bus to announce that the module is online and ready to participate in the network.

### knob::update_rotate()

The knob::update_rotate() function processes raw quadrature inputs from a rotary encoder (knob) and updates the rotation variable accordingly.

### readInput()

The readInput() helper function abstracts the key matrix scanning process. It reads the state of the key matrix (via GPIO pins) and returns a composite representation (e.g., a bitmask) of which keys are pressed.

### autoDetection()

The autoDetection() function is responsible for determining the operational mode (sender or receiver) of a module in a multi-keyboard configuration.

