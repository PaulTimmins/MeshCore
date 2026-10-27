#include "PikaToy.h"

#include <driver/gpio.h>
#include <helpers/TxtDataHelpers.h>

#define PIKA_CFG_MAGIC     0x50494B34   // 'PIK4'
#define PIKA_CFG_FILE      "/pika_cfg"

#define PAW_DEBOUNCE_MILLIS   40
#define REPLY_MAX             150

PikaToy pika;

static const char* EVENT_NAMES[MON_EV_COUNT] = { "newap", "changed", "first", "missing", "back" };
static const char* ACTION_NAMES[] = { "none", "say", "cheeks" };

void notifyMonitorEvent(MonitorEvent ev) {
  pika.onEvent(ev);
}

static int parseAction(const char* s) {
  for (int i = 0; i <= PIKA_ACT_MAX; i++) if (strcmp(s, ACTION_NAMES[i]) == 0) return i;
  return -1;
}

static char* skipSpaces(char* sp) {
  while (*sp == ' ') sp++;
  return sp;
}

PikaToy::PikaToy() {
  _fs = NULL;
  _q_head = _q_count = 0;
  _pressing = false;
  _release_at = _next_allowed = 0;
  _repeat_pending = false;
  _last_activity = 0;
  _mute_until = 0;
  _nag_acked = false;
  _next_nag = 0;
  _last_missing = 0;
  _pending_paw = 0;
  _sense_ignore_until = 0;
  for (int i = 0; i < 2; i++) {
    _paw_pins[i] = -1;
    _paw_state[i] = false;
    _paw_changed[i] = 0;
  }
  setDefaults();
}

void PikaToy::setDefaults() {
  memset(&_cfg, 0, sizeof(_cfg));
  _cfg.magic = PIKA_CFG_MAGIC;
  _cfg.enabled = 1;
  _cfg.active_high = 1;
  _cfg.pin_say = PIKA_PIN_SAY;
  _cfg.pin_cheeks = PIKA_PIN_CHEEKS;
  _cfg.paw_say = PIKA_PAW_SAY;
  _cfg.paw_cheeks = PIKA_PAW_CHEEKS;
  _cfg.paw_passthrough = 1;
  _cfg.channel_cmds = 1;
  _cfg.press_ms = 250;
  _cfg.gap_ms = 3000;
  _cfg.nag_mins = 30;
  _cfg.wake_idle_secs = 60;    // the toy's first press after idling only wakes it (zap)
  _cfg.wake_gap_ms = 4500;   // must outlast the wake-up zap sound (~3s)
  // WiFi events light the cheeks, beacon events say "pika" (which also lights the cheeks)
  _cfg.react[MON_EV_NEW_AP] = PIKA_ACT_CHEEKS;
  _cfg.react[MON_EV_CHANGED_AP] = PIKA_ACT_CHEEKS;
  _cfg.react[MON_EV_BEACON_NEW] = PIKA_ACT_SAY;
  _cfg.react[MON_EV_BEACON_MISSING] = PIKA_ACT_SAY;
  _cfg.react[MON_EV_BEACON_BACK] = PIKA_ACT_SAY;
  _cfg.direct = PIKA_DRIVE_DIRECT;
}

void PikaToy::loadConfig() {
  File f = _fs->open(PIKA_CFG_FILE, "r");
  if (f) {
    PikaConfig tmp;
    int n = f.read((uint8_t *) &tmp, sizeof(tmp));
    f.close();
    if (n == sizeof(tmp) && tmp.magic == PIKA_CFG_MAGIC) {
      for (int i = 0; i < MON_EV_COUNT; i++) if (tmp.react[i] > PIKA_ACT_MAX) tmp.react[i] = PIKA_ACT_NONE;
      _cfg = tmp;
      return;
    }
  }
  setDefaults();
}

void PikaToy::saveConfig() {
  File f = _fs->open(PIKA_CFG_FILE, "w", true);
  if (f) {
    f.write((const uint8_t *) &_cfg, sizeof(_cfg));
    f.close();
  }
}

void PikaToy::begin(FILESYSTEM* fs) {
  _fs = fs;
  loadConfig();
  applyPins();
}

// reject pins used by the board (LoRa, display, button), flash pins, and invalid ones
bool PikaToy::pinAllowed(int pin, bool output) const {
  if (pin < 0) return true;   // -1 = unused
  if (!GPIO_IS_VALID_GPIO((gpio_num_t)pin)) return false;
  if (output && !GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)pin)) return false;
#if CONFIG_IDF_TARGET_ESP32
  if (pin >= 6 && pin <= 11) return false;    // SPI flash
#elif CONFIG_IDF_TARGET_ESP32S3
  if (pin >= 26 && pin <= 32) return false;   // SPI flash / PSRAM
#endif
  const int reserved[] = {
#ifdef P_LORA_NSS
    P_LORA_NSS,
#endif
#ifdef P_LORA_RESET
    P_LORA_RESET,
#endif
#ifdef P_LORA_SCLK
    P_LORA_SCLK,
#endif
#ifdef P_LORA_MISO
    P_LORA_MISO,
#endif
#ifdef P_LORA_MOSI
    P_LORA_MOSI,
#endif
#ifdef P_LORA_DIO_0
    P_LORA_DIO_0,
#endif
#ifdef P_LORA_DIO_1
    P_LORA_DIO_1,
#endif
#ifdef P_LORA_BUSY
    P_LORA_BUSY,
#endif
#ifdef P_LORA_TX_LED
    P_LORA_TX_LED,
#endif
#ifdef PIN_BOARD_SDA
    PIN_BOARD_SDA,
#endif
#ifdef PIN_BOARD_SCL
    PIN_BOARD_SCL,
#endif
#ifdef PIN_OLED_RESET
    PIN_OLED_RESET,
#endif
#ifdef PIN_USER_BTN
    PIN_USER_BTN,
#endif
#ifdef PIN_VBAT_READ
    PIN_VBAT_READ,
#endif
    -1
  };
  for (int i = 0; reserved[i] >= 0; i++) {
    if (reserved[i] == pin) return false;
  }
  return true;
}

void PikaToy::setPin(int8_t pin, bool on) {
  if (pin < 0) return;
  if (_cfg.direct) {
    if (on) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);   // pull the contact to GND, like the paw switch does
    } else {
      pinMode(pin, INPUT);    // release: high impedance, the toy's own pull-up takes over
    }
    return;
  }
  digitalWrite(pin, (on == (_cfg.active_high != 0)) ? HIGH : LOW);
}

void PikaToy::applyPins() {
  if (_cfg.direct) {
    if (_cfg.pin_say >= 0)    pinMode(_cfg.pin_say, INPUT);
    if (_cfg.pin_cheeks >= 0) pinMode(_cfg.pin_cheeks, INPUT);
  } else {
    if (_cfg.pin_say >= 0)    { pinMode(_cfg.pin_say, OUTPUT);    setPin(_cfg.pin_say, false); }
    if (_cfg.pin_cheeks >= 0) { pinMode(_cfg.pin_cheeks, OUTPUT); setPin(_cfg.pin_cheeks, false); }
  }

  // direct mode senses paw squeezes on the same pins it drives
  _paw_pins[0] = _cfg.direct ? _cfg.pin_say : _cfg.paw_say;
  _paw_pins[1] = _cfg.direct ? _cfg.pin_cheeks : _cfg.paw_cheeks;
  for (int i = 0; i < 2; i++) {
    if (_paw_pins[i] >= 0) {
      if (!_cfg.direct) pinMode(_paw_pins[i], INPUT_PULLUP);   // switch to GND
      _paw_state[i] = digitalRead(_paw_pins[i]) == LOW;
      _paw_changed[i] = millis();
    }
  }
}

void PikaToy::releasePins() {
  setPin(_cfg.pin_say, false);
  setPin(_cfg.pin_cheeks, false);
  _pressing = false;
}

bool PikaToy::isMuted() const {
  return _mute_until && (long)(_mute_until - millis()) > 0;
}

void PikaToy::enqueue(uint8_t action) {
  if (action == PIKA_ACT_NONE || action > PIKA_ACT_MAX) return;
  if (_q_count > 0 && _queue[(_q_head + _q_count - 1) % PIKA_QUEUE_SIZE] == action) return;   // coalesce repeats
  if (_q_count >= PIKA_QUEUE_SIZE) return;
  _queue[(_q_head + _q_count) % PIKA_QUEUE_SIZE] = action;
  _q_count++;
}

void PikaToy::onEvent(MonitorEvent ev) {
  if (ev == MON_EV_BEACON_MISSING) {   // a new disappearance re-arms the nag reminder
    _nag_acked = false;
    if (_cfg.nag_mins > 0) _next_nag = millis() + (unsigned long)_cfg.nag_mins * 60000UL;
  }
  if (!_cfg.enabled || isMuted() || ev >= MON_EV_COUNT) return;
  enqueue(_cfg.react[ev]);
}

void PikaToy::doAction(uint8_t action) {
  if (!_cfg.enabled) return;
  enqueue(action);
}

int PikaToy::popPawEvent() {
  int ev = _pending_paw;
  _pending_paw = 0;
  return ev;
}

void PikaToy::pollPaws() {
  if (_cfg.direct && (_pressing || (long)(millis() - _sense_ignore_until) < 0)) return;   // our own press

  for (int i = 0; i < 2; i++) {
    if (_paw_pins[i] < 0) continue;
    bool pressed = digitalRead(_paw_pins[i]) == LOW;
    if (pressed != _paw_state[i] && millis() - _paw_changed[i] > PAW_DEBOUNCE_MILLIS) {
      _paw_state[i] = pressed;
      _paw_changed[i] = millis();
      if (pressed) {
        if (_cfg.paw_passthrough && !_cfg.direct) enqueue(i == 0 ? PIKA_ACT_SAY : PIKA_ACT_CHEEKS);   // direct: toy saw it already
        if (i == 1) _nag_acked = true;   // cheeks paw = acknowledge missing-beacon reminder
        _pending_paw = (i == 0) ? PIKA_PAW_EVENT_SAY : PIKA_PAW_EVENT_CHEEKS;
        _last_activity = millis();   // a real squeeze wakes the toy too
      }
    }
  }
}

void PikaToy::loop(int missing_beacons) {
  pollPaws();

  // missing-beacon nag: repeat cheeks every nag_mins while any beacon is missing, until acknowledged
  if (missing_beacons == 0) {
    _nag_acked = false;
    _next_nag = 0;
  } else if (_cfg.nag_mins > 0 && !_nag_acked) {
    if (_next_nag == 0) {
      _next_nag = millis() + (unsigned long)_cfg.nag_mins * 60000UL;
    } else if ((long)(millis() - _next_nag) >= 0) {
      _next_nag = millis() + (unsigned long)_cfg.nag_mins * 60000UL;
      if (_cfg.enabled && !isMuted()) enqueue(_cfg.react[MON_EV_BEACON_MISSING]);
    }
  }
  _last_missing = missing_beacons;

  // press / release state machine
  if (_pressing) {
    if ((long)(millis() - _release_at) >= 0) {
      releasePins();
      _next_allowed = millis() + (_repeat_pending ? _cfg.wake_gap_ms : _cfg.gap_ms);
      _sense_ignore_until = millis() + 300;   // let the line settle before sensing again
      _last_activity = millis();
    }
  } else if (_repeat_pending && (long)(millis() - _next_allowed) >= 0) {
    _repeat_pending = false;   // toy is awake now, the real 'say' press
    setPin(_cfg.pin_say, true);
    _pressing = true;
    _release_at = millis() + _cfg.press_ms;
  } else if (_q_count > 0 && (long)(millis() - _next_allowed) >= 0) {
    uint8_t action = _queue[_q_head];
    _q_head = (_q_head + 1) % PIKA_QUEUE_SIZE;
    _q_count--;
    if (action == PIKA_ACT_SAY && _cfg.wake_idle_secs > 0 &&
        (_last_activity == 0 || millis() - _last_activity > (unsigned long)_cfg.wake_idle_secs * 1000UL)) {
      _repeat_pending = true;   // this press only wakes the toy, press again after wake_gap_ms
    }
    setPin(action == PIKA_ACT_SAY ? _cfg.pin_say : _cfg.pin_cheeks, true);   // only ever one contact at a time
    _pressing = true;
    _release_at = millis() + _cfg.press_ms;
  }
}

/* ------------------------------ CLI ------------------------------ */

void PikaToy::formatStatus(char* reply) {
  char mute[16] = "";
  if (isMuted()) sprintf(mute, " MUTED %lum", (unsigned long)((_mute_until - millis()) / 60000UL + 1));
  snprintf(reply, REPLY_MAX, "%s%s %s pins:%d/%d paws:%d/%d press:%dms gap:%dms nag:%dm%s chan:%s",
           _cfg.enabled ? "ON" : "OFF", mute, _cfg.direct ? "direct" : "opto",
           _cfg.pin_say, _cfg.pin_cheeks, _paw_pins[0], _paw_pins[1],
           _cfg.press_ms, _cfg.gap_ms, _cfg.nag_mins,
           (_last_missing > 0 && _nag_acked) ? "(acked)" : "", _cfg.channel_cmds ? "on" : "off");
}

bool PikaToy::handleCommand(char* command, char* reply) {
  if (strcmp(command, "pika") == 0) {
    formatStatus(reply);
    return true;
  }
  if (memcmp(command, "pika ", 5) != 0) return false;

  char* cmd = skipSpaces(&command[5]);
  char* arg = strchr(cmd, ' ');
  if (arg) {
    *arg = 0;
    arg = skipSpaces(arg + 1);
  } else {
    arg = &cmd[strlen(cmd)];   // empty string
  }

  if (strcmp(cmd, "status") == 0) {
    formatStatus(reply);

  } else if (strcmp(cmd, "help") == 0) {
    strcpy(reply, "pika say|cheeks|on|off|mute <m>|unmute|ack|react [ev act]|wake|mode|pins <s> <c>|paws|press|gap|nag|level|passthru|chan");

  } else if (strcmp(cmd, "say") == 0 || strcmp(cmd, "cheeks") == 0) {
    if (_cfg.pin_say < 0 && _cfg.pin_cheeks < 0) {
      strcpy(reply, "Err - no output pins set, see 'pika pins'");
    } else {
      doAction(parseAction(cmd));
      strcpy(reply, _cfg.enabled ? "OK" : "Err - pika is off");
    }

  } else if (strcmp(cmd, "on") == 0 || strcmp(cmd, "off") == 0) {
    _cfg.enabled = strcmp(cmd, "on") == 0;
    if (!_cfg.enabled) { _q_count = 0; _repeat_pending = false; releasePins(); }
    saveConfig();
    strcpy(reply, "OK");

  } else if (strcmp(cmd, "mute") == 0) {
    int v = atoi(arg);
    if (v < 1 || v > 10080) {
      strcpy(reply, "Err - pika mute <1..10080 mins>");
    } else {
      _mute_until = millis() + (unsigned long)v * 60000UL;
      _q_count = 0;
      _repeat_pending = false;
      sprintf(reply, "OK - muted for %d mins", v);
    }

  } else if (strcmp(cmd, "unmute") == 0) {
    _mute_until = 0;
    strcpy(reply, "OK");

  } else if (strcmp(cmd, "ack") == 0) {
    _nag_acked = true;
    strcpy(reply, "OK - missing-beacon reminder silenced until next missing beacon");

  } else if (strcmp(cmd, "react") == 0) {
    if (*arg == 0) {
      int len = 0;
      for (int i = 0; i < MON_EV_COUNT; i++) {
        len += snprintf(&reply[len], REPLY_MAX - len, "%s%s:%s", i ? " " : "", EVENT_NAMES[i], ACTION_NAMES[_cfg.react[i] <= PIKA_ACT_MAX ? _cfg.react[i] : 0]);
      }
    } else {
      char* act = strchr(arg, ' ');
      if (act) { *act = 0; act = skipSpaces(act + 1); }
      int ev = -1;
      for (int i = 0; i < MON_EV_COUNT; i++) if (strcmp(arg, EVENT_NAMES[i]) == 0) ev = i;
      int a = act ? parseAction(act) : -1;
      if (ev < 0 || a < 0) {
        strcpy(reply, "Err - pika react <newap|changed|first|missing|back> <none|say|cheeks>");
      } else {
        _cfg.react[ev] = a;
        saveConfig();
        strcpy(reply, "OK");
      }
    }

  } else if (strcmp(cmd, "pins") == 0 || strcmp(cmd, "paws") == 0) {
    bool outputs = strcmp(cmd, "pins") == 0;
    if (*arg == 0) {
      if (outputs) sprintf(reply, "outputs say:%d cheeks:%d (-1 = unused)", _cfg.pin_say, _cfg.pin_cheeks);
      else sprintf(reply, "paw inputs say:%d cheeks:%d (-1 = unused)", _cfg.paw_say, _cfg.paw_cheeks);
    } else {
      int a = -1, b = -1;
      if (sscanf(arg, "%d %d", &a, &b) != 2 || !pinAllowed(a, outputs) || !pinAllowed(b, outputs) || (a >= 0 && a == b)) {
        strcpy(reply, "Err - two different free GPIOs (or -1), eg. pika pins 32 33");
      } else {
        releasePins();
        // return old pins to inputs, so nothing stays driven
        int8_t old[2] = { outputs ? _cfg.pin_say : _cfg.paw_say, outputs ? _cfg.pin_cheeks : _cfg.paw_cheeks };
        for (int i = 0; i < 2; i++) if (old[i] >= 0) pinMode(old[i], INPUT);
        if (outputs) { _cfg.pin_say = a; _cfg.pin_cheeks = b; }
        else { _cfg.paw_say = a; _cfg.paw_cheeks = b; }
        saveConfig();
        applyPins();
        strcpy(reply, "OK");
      }
    }

  } else if (strcmp(cmd, "mode") == 0) {
    if (strcmp(arg, "direct") == 0 || strcmp(arg, "opto") == 0) {
      releasePins();
      int8_t all[4] = { _cfg.pin_say, _cfg.pin_cheeks, _cfg.paw_say, _cfg.paw_cheeks };
      for (int i = 0; i < 4; i++) if (all[i] >= 0) pinMode(all[i], INPUT);   // nothing driven during switch
      _cfg.direct = strcmp(arg, "direct") == 0;
      saveConfig();
      applyPins();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "mode: %s (pika mode direct|opto)", _cfg.direct ? "direct" : "opto");
    }

  } else if (strcmp(cmd, "wake") == 0) {
    int idle = -1, gap = -1;
    int n = sscanf(arg, "%d %d", &idle, &gap);
    if (n < 1) {
      sprintf(reply, "wake: idle %ds, gap %dms (0 = off). pika wake <idle_secs> [gap_ms]", _cfg.wake_idle_secs, _cfg.wake_gap_ms);
    } else if (idle < 0 || idle > 3600 || (n == 2 && (gap < 200 || gap > 10000))) {
      strcpy(reply, "Err - idle 0..3600 secs, gap 200..10000 ms");
    } else {
      _cfg.wake_idle_secs = idle;
      if (n == 2) _cfg.wake_gap_ms = gap;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "press") == 0) {
    int v = atoi(arg);
    if (v < 20 || v > 5000) {
      sprintf(reply, "press:%dms (set 20..5000)", _cfg.press_ms);
    } else {
      _cfg.press_ms = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "gap") == 0) {
    int v = atoi(arg);
    if (v < 100 || v > 60000) {
      sprintf(reply, "gap:%dms (set 100..60000)", _cfg.gap_ms);
    } else {
      _cfg.gap_ms = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "nag") == 0) {
    if (*arg == 0) {
      sprintf(reply, "nag every %d mins while beacons missing (0 = off)", _cfg.nag_mins);
    } else {
      int v = atoi(arg);
      if (v < 0 || v > 1440) {
        strcpy(reply, "Err - 0..1440 mins");
      } else {
        _cfg.nag_mins = v;
        _next_nag = 0;
        saveConfig();
        strcpy(reply, "OK");
      }
    }

  } else if (strcmp(cmd, "level") == 0) {
    if (strcmp(arg, "high") == 0 || strcmp(arg, "low") == 0) {
      releasePins();
      _cfg.active_high = strcmp(arg, "high") == 0;
      releasePins();   // re-apply idle level with new polarity
      saveConfig();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "active level: %s (pika level high|low)", _cfg.active_high ? "high" : "low");
    }

  } else if (strcmp(cmd, "passthru") == 0) {
    if (strcmp(arg, "on") == 0 || strcmp(arg, "off") == 0) {
      _cfg.paw_passthrough = strcmp(arg, "on") == 0;
      saveConfig();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "paw passthrough: %s", _cfg.paw_passthrough ? "on" : "off");
    }

  } else if (strcmp(cmd, "chan") == 0) {
    if (strcmp(arg, "on") == 0 || strcmp(arg, "off") == 0) {
      _cfg.channel_cmds = strcmp(arg, "on") == 0;
      saveConfig();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "channel commands (!pika !cheeks !status): %s", _cfg.channel_cmds ? "on" : "off");
    }

  } else {
    strcpy(reply, "Err - unknown. Try 'pika help'");
  }
  return true;
}
