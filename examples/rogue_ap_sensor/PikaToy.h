#pragma once

#include <Arduino.h>
#include <helpers/IdentityStore.h>   // for FILESYSTEM
#include "MonitorEvents.h"

/*
 * Drives a toy's two "dry contact" buttons (eg. a Pikachu's 'say pika' and 'cheeks light up' paws)
 * through optocouplers, and optionally reads the toy's paw switches as inputs:
 *
 * Two drive modes:
 *  - opto:   GPIO out -> 330R -> optocoupler LED -> GND (opto transistor across the toy's contact),
 *            paw switches moved to separate GPIO inputs (to GND, internal pull-up) and passed through
 *  - direct: for toys whose contacts are active-low at <= 3.3V logic, with a shared ground.
 *            GPIO -> 1k -> toy contact pad. Idle = high impedance input (also senses real paw squeezes),
 *            press = drive LOW (open-drain style, never drives HIGH)
 *
 * Paw squeezes raise mesh actions (status / acknowledge).
 */

#ifndef PIKA_DRIVE_DIRECT
  #define PIKA_DRIVE_DIRECT  0
#endif

#ifndef PIKA_PIN_SAY
  #define PIKA_PIN_SAY       -1   // output: closes the toy's 'say' contact
#endif
#ifndef PIKA_PIN_CHEEKS
  #define PIKA_PIN_CHEEKS    -1   // output: closes the toy's 'cheeks' contact
#endif
#ifndef PIKA_PAW_SAY
  #define PIKA_PAW_SAY       -1   // input: paw switch for 'say'
#endif
#ifndef PIKA_PAW_CHEEKS
  #define PIKA_PAW_CHEEKS    -1   // input: paw switch for 'cheeks'
#endif

#define PIKA_ACT_NONE     0
#define PIKA_ACT_SAY      1
#define PIKA_ACT_CHEEKS   2
#define PIKA_ACT_MAX      2     // the toy's contacts must never be pressed together

#define PIKA_PAW_EVENT_SAY      1
#define PIKA_PAW_EVENT_CHEEKS   2

#define PIKA_QUEUE_SIZE   4

struct PikaConfig {
  uint32_t magic;
  uint8_t  enabled;
  uint8_t  active_high;      // output level that turns the optocoupler on
  int8_t   pin_say, pin_cheeks;
  int8_t   paw_say, paw_cheeks;
  uint8_t  paw_passthrough;  // paw press also presses the toy's contact
  uint8_t  channel_cmds;     // react to !pika / !cheeks / !status on the report channel
  uint16_t press_ms;         // how long a contact is held closed
  uint16_t gap_ms;           // min time between presses (let the toy finish its sound/animation)
  uint16_t nag_mins;         // repeat the 'missing' reaction while beacons are missing (0 = off)
  uint8_t  react[MON_EV_COUNT];   // PIKA_ACT_* per MonitorEvent
  uint8_t  direct;           // 1 = direct (open-drain, shared pin senses paws), 0 = opto
};

class PikaToy {
public:
  PikaToy();

  void begin(FILESYSTEM* fs);
  // missing_beacons: current count, drives the nag reminder
  void loop(int missing_beacons);

  void onEvent(MonitorEvent ev);
  void doAction(uint8_t action);   // manual / channel triggered (ignores mute)

  // returns PIKA_PAW_EVENT_* when a paw was pressed since last call (0 = none)
  int popPawEvent();

  bool handleCommand(char* command, char* reply);
  bool channelCmdsEnabled() const { return _cfg.channel_cmds != 0; }

private:
  FILESYSTEM* _fs;
  PikaConfig _cfg;

  uint8_t _queue[PIKA_QUEUE_SIZE];
  int _q_head, _q_count;
  bool _pressing;
  unsigned long _release_at, _next_allowed;

  unsigned long _mute_until;
  bool _nag_acked;
  unsigned long _next_nag;
  int _last_missing;

  // paw debounce
  int8_t _paw_pins[2];
  bool _paw_state[2];
  unsigned long _paw_changed[2];
  int _pending_paw;
  unsigned long _sense_ignore_until;   // direct mode: ignore our own presses

  void setDefaults();
  void loadConfig();
  void saveConfig();
  void applyPins();
  void releasePins();
  void setPin(int8_t pin, bool on);
  void enqueue(uint8_t action);
  bool isMuted() const;
  bool pinAllowed(int pin, bool output) const;
  void pollPaws();
  void formatStatus(char* reply);
};

extern PikaToy pika;
