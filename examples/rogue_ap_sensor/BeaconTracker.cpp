#include "BeaconTracker.h"
#include "RogueApScanner.h"
#include "MonitorEvents.h"

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Utils.h>
#include <helpers/TxtDataHelpers.h>

#define BEACON_CFG_MAGIC     0x42434E32   // 'BCN2'
#define BEACON_TABLE_MAGIC   0x42434E54   // 'BCNT'
#define BEACON_CFG_FILE      "/beacon_cfg"
#define BEACON_TABLE_FILE    "/beacons"

#define CHECK_INTERVAL_MILLIS     2000
#define SCAN_INTERVAL_MS          100
#define DEFAULT_DUTY_PCT          90
#define SCAN_RESTART_MILLIS       (30UL*60*1000)   // periodically restart the scan, as a safety net
#define MAX_EVENTS_PER_CHECK      6
#define REPLY_MAX                 150
#define LIST_PAGE_SIZE            3

BeaconTracker beacons;

static volatile bool scan_ended = false;

static void onScanComplete(BLEScanResults results) {
  scan_ended = true;   // scan stopped unexpectedly (duration is 0 = forever), loop() will restart it
}

class TrackerAdvertCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
  void onResult(BLEAdvertisedDevice dev) override {
    std::string name;
    if (dev.haveName()) name = dev.getName();
    beacons.onAdvert(*dev.getAddress().getNative(), dev.getAddressType(), dev.getRSSI(), name.c_str());
  }
};
static TrackerAdvertCallbacks advert_callbacks;

static void macToStr(char* dest, const uint8_t* mac) {   // dest: 18 chars
  sprintf(dest, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void formatAge(char* dest, uint32_t secs) {
  if (secs < 60) {
    sprintf(dest, "%lus", (unsigned long)secs);
  } else if (secs < 3600) {
    sprintf(dest, "%lum", (unsigned long)(secs / 60));
  } else if (secs < 86400) {
    sprintf(dest, "%luh%02lum", (unsigned long)(secs / 3600), (unsigned long)((secs / 60) % 60));
  } else {
    sprintf(dest, "%lud%02luh", (unsigned long)(secs / 86400), (unsigned long)((secs / 3600) % 24));
  }
}

// copy name for display: non-printables replaced
static void nameForDisplay(char* dest, int dest_size, const char* name) {
  int i = 0;
  for (; name[i] && i < dest_size - 1; i++) {
    char c = name[i];
    dest[i] = (c < ' ' || c > '~') ? '?' : c;
  }
  dest[i] = 0;
}

static char* skipSpaces(char* sp) {
  while (*sp == ' ') sp++;
  return sp;
}

/* ------------------------------------------------------------------------------- */

BeaconTracker::BeaconTracker() {
  _fs = NULL;
  _rtc = NULL;
  _out = NULL;
  _lock = portMUX_INITIALIZER_UNLOCKED;
  _num_beacons = 0;
  _adverts_seen = 0;
  _table_full = _table_changed = _table_full_reported = false;
  _ble_ready = _scanning = false;
  _next_check_millis = _next_restart_millis = _scan_started_millis = 0;
  setDefaults();
}

void BeaconTracker::setDefaults() {
  memset(&_cfg, 0, sizeof(_cfg));
  _cfg.magic = BEACON_CFG_MAGIC;
  strcpy(_cfg.prefix, "RF");
  _cfg.timeout_mins = 60;
  _cfg.min_rssi = 0;
  _cfg.enabled = 1;
  _cfg.active_scan = 1;
  _cfg.key_by_name = 0;
}

void BeaconTracker::begin(FILESYSTEM* fs, mesh::RTCClock* rtc, RogueApScanner* out) {
  _fs = fs;
  _rtc = rtc;
  _out = out;

  loadConfig();
  loadTable();

  if (_cfg.enabled) startScan();
  _next_check_millis = millis() + CHECK_INTERVAL_MILLIS;
}

/* ------------------------------ persistence ------------------------------ */

void BeaconTracker::loadConfig() {
  File f = _fs->open(BEACON_CFG_FILE, "r");
  if (f) {
    BeaconConfig tmp;
    int n = f.read((uint8_t *) &tmp, sizeof(tmp));
    f.close();
    if (n == sizeof(tmp) && tmp.magic == BEACON_CFG_MAGIC) {
      tmp.prefix[BEACON_PREFIX_LEN] = 0;
      _cfg = tmp;
      return;
    }
  }
  setDefaults();
}

void BeaconTracker::saveConfig() {
  File f = _fs->open(BEACON_CFG_FILE, "w", true);
  if (f) {
    f.write((const uint8_t *) &_cfg, sizeof(_cfg));
    f.close();
  }
}

void BeaconTracker::loadTable() {
  _num_beacons = 0;
  File f = _fs->open(BEACON_TABLE_FILE, "r");
  if (!f) return;

  uint32_t magic = 0;
  uint16_t count = 0;
  if (f.read((uint8_t *) &magic, 4) == 4 && magic == BEACON_TABLE_MAGIC && f.read((uint8_t *) &count, 2) == 2) {
    unsigned long now_ms = millis();
    while (_num_beacons < count && _num_beacons < BEACON_MAX) {
      BeaconEntry* b = &_beacons[_num_beacons];
      if (f.read((uint8_t *) b, sizeof(BeaconEntry)) != sizeof(BeaconEntry)) break;
      b->name[BEACON_NAME_LEN] = 0;
      b->flags = 0;
      b->last_seen_ms = now_ms;   // reboot restarts the missing timer (missing ones stay missing)
      _num_beacons++;
    }
  }
  f.close();
}

void BeaconTracker::saveTable() {
  BeaconEntry* copy = (BeaconEntry *) malloc(sizeof(BeaconEntry) * BEACON_MAX);
  if (copy == NULL) return;

  portENTER_CRITICAL(&_lock);
  int count = _num_beacons;
  memcpy(copy, _beacons, sizeof(BeaconEntry) * count);
  _table_changed = false;
  portEXIT_CRITICAL(&_lock);

  File f = _fs->open(BEACON_TABLE_FILE, "w", true);
  if (f) {
    uint32_t magic = BEACON_TABLE_MAGIC;
    uint16_t n = count;
    f.write((const uint8_t *) &magic, 4);
    f.write((const uint8_t *) &n, 2);
    f.write((const uint8_t *) copy, sizeof(BeaconEntry) * count);
    f.close();
  }
  free(copy);
}

/* ------------------------------ scanning ------------------------------ */

void BeaconTracker::startScan() {
  if (!_ble_ready) {
    BLEDevice::init("");
    _ble_ready = true;
  }
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&advert_callbacks, true /*want duplicates*/, true /*parse*/);
  scan->setActiveScan(_cfg.active_scan != 0);
  int duty = _cfg.duty_pct ? _cfg.duty_pct : DEFAULT_DUTY_PCT;
  scan->setInterval(SCAN_INTERVAL_MS);
  scan->setWindow(SCAN_INTERVAL_MS * duty / 100);   // listen this many ms out of every 100ms
  scan_ended = false;
  _scanning = scan->start(0 /*forever*/, onScanComplete, false);
  _scan_started_millis = millis();
  _next_restart_millis = millis() + (_scanning ? SCAN_RESTART_MILLIS : 60000UL);   // retry soon if start failed
}

void BeaconTracker::stopScan() {
  if (_ble_ready && _scanning) {
    BLEDevice::getScan()->stop();
  }
  _scanning = false;
}

// NOTE: caller must hold _lock
int BeaconTracker::findLocked(const uint8_t* mac, const char* name) const {
  if (_cfg.key_by_name && name && name[0]) {
    for (int i = 0; i < _num_beacons; i++) {
      if (strcmp(_beacons[i].name, name) == 0) return i;
    }
    return -1;
  }
  for (int i = 0; i < _num_beacons; i++) {
    if (memcmp(_beacons[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

// does name start with any of the comma-separated prefixes? NOTE: caller must hold _lock
bool BeaconTracker::nameMatchesLocked(const char* name) const {
  if (name == NULL || name[0] == 0) return false;
  const char* p = _cfg.prefix;
  while (*p) {
    const char* end = strchr(p, ',');
    int plen = end ? end - p : strlen(p);
    if (plen > 0 && strncmp(name, p, plen) == 0) return true;
    if (end == NULL) break;
    p = end + 1;
  }
  return false;
}

// called from the BT task, for every advertisement received
void BeaconTracker::onAdvert(const uint8_t* mac, uint8_t addr_type, int rssi, const char* name) {
  unsigned long now_ms = millis();

  portENTER_CRITICAL(&_lock);
  bool name_match = nameMatchesLocked(name);
  _adverts_seen++;
  int i = findLocked(mac, name_match ? name : NULL);
  if (i >= 0) {
    BeaconEntry* b = &_beacons[i];
    if (b->state == BEACON_STATE_MISSING) {
      b->state = BEACON_STATE_PRESENT;
      b->flags |= BEACON_FLAG_BACK;
      b->last_seen = (now_ms - b->last_seen_ms) / 1000;   // stash: how long it was gone (secs)
    }
    b->last_seen_ms = now_ms;
    b->rssi = rssi;
    b->sightings++;
    if (name_match && strncmp(b->name, name, BEACON_NAME_LEN) != 0) {
      StrHelper::strncpy(b->name, name, sizeof(b->name));
      _table_changed = true;
    }
    if (memcmp(b->mac, mac, 6) != 0) {   // key_by_name, and the beacon rotated its address
      memcpy(b->mac, mac, 6);
      b->addr_type = addr_type;
      _table_changed = true;
    }
  } else if (name_match && (_cfg.min_rssi == 0 || rssi >= _cfg.min_rssi)) {
    if (_num_beacons < BEACON_MAX) {
      BeaconEntry* b = &_beacons[_num_beacons++];
      memset(b, 0, sizeof(*b));
      memcpy(b->mac, mac, 6);
      b->addr_type = addr_type;
      b->state = BEACON_STATE_PRESENT;
      b->rssi = rssi;
      b->flags = BEACON_FLAG_NEW;
      StrHelper::strncpy(b->name, name, sizeof(b->name));
      b->sightings = 1;
      b->last_seen_ms = now_ms;
      _table_changed = true;
    } else {
      _table_full = true;
    }
  }
  portEXIT_CRITICAL(&_lock);
}

void BeaconTracker::report(const char* text, bool alert) {
  if (_out == NULL) return;
  _out->queueText(text);
  if (alert) _out->queueAlert(text);
}

void BeaconTracker::checkBeacons() {
  enum EventType { EV_NEW, EV_BACK, EV_MISSING };
  struct Event { EventType type; BeaconEntry b; uint32_t gone_secs; };
  Event events[MAX_EVENTS_PER_CHECK];
  int num_events = 0;

  uint32_t now = _rtc->getCurrentTime();
  unsigned long now_ms = millis();
  unsigned long timeout_ms = (unsigned long)_cfg.timeout_mins * 60000UL;
  bool changed = false;
  int total = 0;

  portENTER_CRITICAL(&_lock);
  total = _num_beacons;
  for (int i = 0; i < _num_beacons && num_events < MAX_EVENTS_PER_CHECK; i++) {
    BeaconEntry* b = &_beacons[i];
    if (b->flags & BEACON_FLAG_NEW) {
      b->flags &= ~BEACON_FLAG_NEW;
      b->first_seen = now - (now_ms - b->last_seen_ms) / 1000;
      events[num_events].type = EV_NEW;
      events[num_events].b = *b;
      events[num_events].gone_secs = 0;
      num_events++;
      changed = true;
    } else if (b->flags & BEACON_FLAG_BACK) {
      b->flags &= ~BEACON_FLAG_BACK;
      events[num_events].type = EV_BACK;
      events[num_events].b = *b;
      events[num_events].gone_secs = b->last_seen;   // stashed by onAdvert()
      num_events++;
      changed = true;
    } else if (b->state == BEACON_STATE_PRESENT && now_ms - b->last_seen_ms > timeout_ms) {
      b->state = BEACON_STATE_MISSING;
      events[num_events].type = EV_MISSING;
      events[num_events].b = *b;
      events[num_events].gone_secs = (now_ms - b->last_seen_ms) / 1000;
      num_events++;
      changed = true;
    }
    if (!(b->flags & BEACON_FLAG_BACK)) {   // keep RTC last_seen fresh (except while a BACK report is pending)
      b->last_seen = now - (now_ms - b->last_seen_ms) / 1000;
    }
  }
  bool full = _table_full;
  _table_full = false;
  if (_table_changed) changed = true;
  portEXIT_CRITICAL(&_lock);

  for (int i = 0; i < num_events; i++) {
    auto e = &events[i];
    char mac[18], name[BEACON_NAME_LEN + 1], age[12], msg[BEACON_MAX_TEXT + 1];
    macToStr(mac, e->b.mac);
    nameForDisplay(name, sizeof(name), e->b.name);
    formatAge(age, e->gone_secs);
    switch (e->type) {
      case EV_NEW:
        snprintf(msg, sizeof(msg), "BEACON FIRST HEARD %s %s %ddBm (%d tracked)", name, mac, e->b.rssi, total);
        report(msg, false);
        notifyMonitorEvent(MON_EV_BEACON_NEW);
        break;
      case EV_BACK:
        snprintf(msg, sizeof(msg), "BEACON BACK %s %s %ddBm, was missing %s", name, mac, e->b.rssi, age);
        report(msg, true);
        notifyMonitorEvent(MON_EV_BEACON_BACK);
        break;
      case EV_MISSING:
        snprintf(msg, sizeof(msg), "BEACON MISSING %s %s not heard for %s (last %ddBm)", name, mac, age, e->b.rssi);
        report(msg, true);
        notifyMonitorEvent(MON_EV_BEACON_MISSING);
        break;
    }
  }

  if (full && !_table_full_reported) {
    _table_full_reported = true;
    char msg[BEACON_MAX_TEXT + 1];
    snprintf(msg, sizeof(msg), "WARN: beacon table full (%d), new beacons not tracked. Use 'beacon forget'", BEACON_MAX);
    report(msg, false);
  }
  if (changed) saveTable();
}

void BeaconTracker::loop() {
  if (!_cfg.enabled) return;

  bool ended = scan_ended && millis() - _scan_started_millis > 5000;   // ignore late 'complete' events from our own stop()
  if (ended || (long)(millis() - _next_restart_millis) >= 0) {   // restart scanning
    stopScan();
    startScan();
  }

  if ((long)(millis() - _next_check_millis) >= 0) {
    _next_check_millis = millis() + CHECK_INTERVAL_MILLIS;
    checkBeacons();
  }
}

/* ------------------------------ CLI ------------------------------ */

void BeaconTracker::getCounts(int& present, int& missing) {
  present = missing = 0;
  portENTER_CRITICAL(&_lock);
  for (int i = 0; i < _num_beacons; i++) {
    if (_beacons[i].state == BEACON_STATE_PRESENT) present++; else missing++;
  }
  portEXIT_CRITICAL(&_lock);
}

void BeaconTracker::formatStatus(char* reply) {
  int present = 0, missing = 0;
  portENTER_CRITICAL(&_lock);
  for (int i = 0; i < _num_beacons; i++) {
    if (_beacons[i].state == BEACON_STATE_PRESENT) present++; else missing++;
  }
  uint32_t adverts = _adverts_seen;
  portEXIT_CRITICAL(&_lock);

  snprintf(reply, REPLY_MAX, "%s %s prefix:\"%s\" key:%s present:%d missing:%d timeout:%dm rssi:%d adverts:%lu",
           _cfg.enabled ? (_scanning ? "SCANNING" : "STOPPED") : "OFF", _cfg.active_scan ? "active" : "passive",
           _cfg.prefix, _cfg.key_by_name ? "name" : "mac", present, missing, _cfg.timeout_mins, _cfg.min_rssi,
           (unsigned long)adverts);
}

void BeaconTracker::formatList(char* reply, uint8_t state_mask, int page) {
  unsigned long now_ms = millis();
  int total = 0;
  portENTER_CRITICAL(&_lock);
  for (int i = 0; i < _num_beacons; i++) if (_beacons[i].state & state_mask) total++;
  portEXIT_CRITICAL(&_lock);

  if (total == 0) {
    strcpy(reply, "(none)");
    return;
  }
  int pages = (total + LIST_PAGE_SIZE - 1) / LIST_PAGE_SIZE;
  if (page < 1) page = 1;
  if (page > pages) page = pages;

  int len = sprintf(reply, "p%d/%d (%d)", page, pages, total);
  int skip = (page - 1) * LIST_PAGE_SIZE, shown = 0;
  for (int i = 0; shown < LIST_PAGE_SIZE; i++) {
    BeaconEntry b;
    portENTER_CRITICAL(&_lock);
    bool valid = i < _num_beacons;
    if (valid) b = _beacons[i];
    portEXIT_CRITICAL(&_lock);
    if (!valid) break;

    if (!(b.state & state_mask)) continue;
    if (skip > 0) { skip--; continue; }

    char hex[13], name[BEACON_NAME_LEN + 1], age[12];
    mesh::Utils::toHex(hex, b.mac, 6);
    nameForDisplay(name, sizeof(name), b.name);
    formatAge(age, (now_ms - b.last_seen_ms) / 1000);
    len += snprintf(&reply[len], REPLY_MAX - len, "\n%s %s %d %s%s", name, hex, b.rssi, age,
                    b.state == BEACON_STATE_MISSING ? " MISSING" : "");
    if (len >= REPLY_MAX) { reply[REPLY_MAX - 1] = 0; break; }
    shown++;
  }
}

// match by exact name (case-insensitive), or hex MAC prefix (min 4 digits). Returns number of matches
int BeaconTracker::resolve(const char* arg, int* idx_out) {
  int n = 0;
  portENTER_CRITICAL(&_lock);
  for (int i = 0; i < _num_beacons; i++) {
    if (strcasecmp(_beacons[i].name, arg) == 0) { *idx_out = i; n++; }
  }
  portEXIT_CRITICAL(&_lock);
  if (n > 0) return n;

  char want[13];
  int digits = 0;
  for (const char* sp = arg; *sp; sp++) {
    char c = *sp;
    if (c == ':' || c == '-') continue;
    if (!mesh::Utils::isHexChar(c) || digits >= 12) return 0;
    want[digits++] = toupper((unsigned char)c);
  }
  want[digits] = 0;
  if (digits < 4) return 0;

  portENTER_CRITICAL(&_lock);
  for (int i = 0; i < _num_beacons; i++) {
    char hex[13];
    mesh::Utils::toHex(hex, _beacons[i].mac, 6);
    if (strncmp(hex, want, digits) == 0) { *idx_out = i; n++; }
  }
  portEXIT_CRITICAL(&_lock);
  return n;
}

bool BeaconTracker::handleCommand(char* command, char* reply) {
  if (strcmp(command, "beacon") == 0) {
    formatStatus(reply);
    return true;
  }
  if (memcmp(command, "beacon ", 7) != 0) return false;

  char* cmd = skipSpaces(&command[7]);
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
    strcpy(reply, "beacon status|list [p]|missing [p]|show <name|mac>|forget <name|mac|missing>|prefix|timeout|rssi|scan|duty|key|on|off|reset");

  } else if (strcmp(cmd, "list") == 0) {
    formatList(reply, BEACON_STATE_PRESENT | BEACON_STATE_MISSING, atoi(arg));

  } else if (strcmp(cmd, "missing") == 0) {
    formatList(reply, BEACON_STATE_MISSING, atoi(arg));

  } else if (strcmp(cmd, "show") == 0) {
    int idx = -1;
    int n = resolve(arg, &idx);
    if (n != 1) {
      strcpy(reply, n > 1 ? "Err - ambiguous" : "Err - not found");
    } else {
      BeaconEntry b;
      portENTER_CRITICAL(&_lock);
      b = _beacons[idx];
      portEXIT_CRITICAL(&_lock);
      char mac[18], name[BEACON_NAME_LEN + 1], seen[12], first[12];
      macToStr(mac, b.mac);
      nameForDisplay(name, sizeof(name), b.name);
      formatAge(seen, (millis() - b.last_seen_ms) / 1000);
      formatAge(first, _rtc->getCurrentTime() - b.first_seen);
      snprintf(reply, REPLY_MAX, "%s %s %s %ddBm last:%s ago first:%s ago heard:%lu",
               name, mac, b.state == BEACON_STATE_MISSING ? "MISSING" : "PRESENT", b.rssi, seen, first,
               (unsigned long)b.sightings);
    }

  } else if (strcmp(cmd, "forget") == 0) {
    if (strcmp(arg, "missing") == 0) {
      int removed = 0;
      portENTER_CRITICAL(&_lock);
      int i = 0;
      while (i < _num_beacons) {
        if (_beacons[i].state == BEACON_STATE_MISSING) {
          _beacons[i] = _beacons[--_num_beacons];
          removed++;
        } else {
          i++;
        }
      }
      portEXIT_CRITICAL(&_lock);
      _table_full_reported = false;
      saveTable();
      sprintf(reply, "OK - %d forgotten", removed);
    } else {
      int idx = -1;
      int n = resolve(arg, &idx);
      if (n != 1) {
        strcpy(reply, n > 1 ? "Err - ambiguous" : "Err - not found");
      } else {
        portENTER_CRITICAL(&_lock);
        _beacons[idx] = _beacons[--_num_beacons];
        portEXIT_CRITICAL(&_lock);
        _table_full_reported = false;
        saveTable();
        strcpy(reply, "OK - forgotten (reported as first heard if seen again)");
      }
    }

  } else if (strcmp(cmd, "prefix") == 0) {
    if (*arg == 0) {
      sprintf(reply, "prefix: \"%s\"", _cfg.prefix);
    } else if (strlen(arg) > BEACON_PREFIX_LEN || strchr(arg, ' ')) {
      sprintf(reply, "Err - max %d chars, no spaces. eg. RF or RF,TG", BEACON_PREFIX_LEN);
    } else {
      portENTER_CRITICAL(&_lock);
      StrHelper::strncpy(_cfg.prefix, arg, sizeof(_cfg.prefix));
      portEXIT_CRITICAL(&_lock);
      saveConfig();
      strcpy(reply, "OK - applies to new beacons, already tracked ones stay tracked");
    }

  } else if (strcmp(cmd, "timeout") == 0) {
    int v = atoi(arg);
    if (v < 5 || v > 10080) {
      strcpy(reply, "Err - 5..10080 mins");
    } else {
      _cfg.timeout_mins = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "rssi") == 0) {
    int v = atoi(arg);
    if (v > 0 || v < -120) {
      strcpy(reply, "Err - -120..-1 dBm, or 0 = off");
    } else {
      _cfg.min_rssi = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "scan") == 0) {
    if (strcmp(arg, "active") == 0 || strcmp(arg, "passive") == 0) {
      _cfg.active_scan = strcmp(arg, "active") == 0;
      saveConfig();
      if (_cfg.enabled) { stopScan(); startScan(); }
      strcpy(reply, "OK");
    } else {
      strcpy(reply, "Err - beacon scan active|passive");
    }

  } else if (strcmp(cmd, "duty") == 0) {
    int v = atoi(arg);
    if (*arg == 0) {
      sprintf(reply, "BLE listen duty: %d%% (lower = less power, may miss short adverts)", _cfg.duty_pct ? _cfg.duty_pct : DEFAULT_DUTY_PCT);
    } else if (v < 5 || v > 100) {
      strcpy(reply, "Err - 5..100 %");
    } else {
      _cfg.duty_pct = v;
      saveConfig();
      if (_cfg.enabled) { stopScan(); startScan(); }
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "key") == 0) {
    if (strcmp(arg, "mac") == 0 || strcmp(arg, "name") == 0) {
      portENTER_CRITICAL(&_lock);
      _cfg.key_by_name = strcmp(arg, "name") == 0;
      portEXIT_CRITICAL(&_lock);
      saveConfig();
      strcpy(reply, "OK");
    } else {
      strcpy(reply, "Err - beacon key mac|name");
    }

  } else if (strcmp(cmd, "on") == 0) {
    _cfg.enabled = 1;
    saveConfig();
    if (!_scanning) startScan();
    strcpy(reply, "OK - scanning");

  } else if (strcmp(cmd, "off") == 0) {
    _cfg.enabled = 0;
    saveConfig();
    stopScan();
    strcpy(reply, "OK - stopped");

  } else if (strcmp(cmd, "reset") == 0) {
    if (strcmp(arg, "confirm") == 0) {
      portENTER_CRITICAL(&_lock);
      _num_beacons = 0;
      portEXIT_CRITICAL(&_lock);
      _table_full_reported = false;
      saveTable();
      strcpy(reply, "OK - all beacons forgotten");
    } else {
      strcpy(reply, "Forgets ALL tracked beacons. Send 'beacon reset confirm'");
    }

  } else {
    strcpy(reply, "Err - unknown. Try 'beacon help'");
  }
  return true;
}
