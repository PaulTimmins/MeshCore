#include "RogueApScanner.h"
#include "MonitorEvents.h"

#ifndef ESP32
  #error "rogue_ap_sensor requires an ESP32 target (needs the on-chip WiFi radio)"
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <Utils.h>
#include <helpers/TxtDataHelpers.h>

#define ROGUE_CFG_MAGIC     0x52474331   // 'RGC1'
#define ROGUE_TABLE_MAGIC   0x52474154   // 'RGAT'
#define ROGUE_CFG_FILE      "/rogue_cfg"
#define ROGUE_TABLE_FILE    "/rogue_aps"

#define SCAN_TIMEOUT_MILLIS   30000
#define REPLY_MAX             150     // CLI replies (remote CLI frame holds ~160 chars)
#define LIST_PAGE_SIZE        3

RogueApScanner rogue;

static const char* authName(uint8_t auth) {
  switch (auth) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/2";
    case WIFI_AUTH_ENTERPRISE:      return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/3";
    case WIFI_AUTH_WAPI_PSK:        return "WAPI";
    case WIFI_AUTH_WPA3_ENT_192:    return "WPA3-E";
  }
  return "?";
}

static void macToHex(char* dest, const uint8_t* mac) {   // dest: 13 chars
  mesh::Utils::toHex(dest, mac, 6);
}

static void macToStr(char* dest, const uint8_t* mac) {   // dest: 18 chars
  sprintf(dest, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// copy SSID for display: non-printables (and quotes) replaced, empty shown as <hidden>
static void ssidForDisplay(char* dest, int dest_size, const char* ssid) {
  if (ssid[0] == 0) {
    StrHelper::strncpy(dest, "<hidden>", dest_size);
    return;
  }
  int i = 0;
  for (; ssid[i] && i < dest_size - 1; i++) {
    char c = ssid[i];
    dest[i] = (c < ' ' || c > '~' || c == '"') ? '?' : c;
  }
  dest[i] = 0;
}

// case-insensitive glob match, supporting '*' and '?'
static bool globMatch(const char* pat, const char* str) {
  const char *star = NULL, *resume = NULL;
  while (*str) {
    if (*pat == '*') {
      star = pat++;
      resume = str;
    } else if (*pat == '?' || tolower((unsigned char)*pat) == tolower((unsigned char)*str)) {
      pat++; str++;
    } else if (star) {
      pat = star + 1;
      str = ++resume;
    } else {
      return false;
    }
  }
  while (*pat == '*') pat++;
  return *pat == 0;
}

// extract hex digits from arg (separators ':', '-', '.' ignored). Returns num digits, or -1 if invalid
static int normaliseHex(char* dest, int dest_size, const char* arg) {
  int n = 0;
  for (const char* sp = arg; *sp; sp++) {
    char c = *sp;
    if (c == ':' || c == '-' || c == '.') continue;
    if (!mesh::Utils::isHexChar(c)) return -1;
    if (n >= dest_size - 1) return -1;
    dest[n++] = toupper((unsigned char)c);
  }
  dest[n] = 0;
  return n;
}

static char* skipSpaces(char* sp) {
  while (*sp == ' ') sp++;
  return sp;
}

/* ------------------------------------------------------------------------------- */

RogueApScanner::RogueApScanner() {
  _fs = NULL;
  _rtc = NULL;
  _aps = NULL;
  _num_aps = _num_cand = 0;
  _learning = _scanning = false;
  _learn_end_millis = 0;
  _scan_start_millis = _next_scan_millis = _next_heartbeat_millis = 0;
  _scan_seq = _last_scan_secs = 0;
  _last_scan_count = _last_scan_new = 0;
  _total_scans = _failed_scans = 0;
  _table_dirty = _table_full_reported = _ota_paused = false;
  _tx_head = _tx_count = 0;
  _tx_dropped = 0;
  _alert_head = _alert_count = 0;
  memset(&_channel, 0, sizeof(_channel));
  setDefaults();
}

void RogueApScanner::setDefaults() {
  memset(&_cfg, 0, sizeof(_cfg));
  _cfg.magic = ROGUE_CFG_MAGIC;
  _cfg.dm_alerts = 1;
  _cfg.confirm_scans = 2;
  _cfg.max_alerts_per_scan = 5;
  _cfg.min_rssi = -90;
  _cfg.interval_mins = 5;
  _cfg.dwell_ms = 300;
  _cfg.heartbeat_hours = 24;
  _cfg.tx_gap_secs = 5;
}

void RogueApScanner::begin(FILESYSTEM* fs, mesh::RTCClock* rtc) {
  _fs = fs;
  _rtc = rtc;

  _aps = (RogueAp *) (psramFound() ? ps_calloc(ROGUE_MAX_APS, sizeof(RogueAp)) : calloc(ROGUE_MAX_APS, sizeof(RogueAp)));

  bool fresh = !_fs->exists(ROGUE_CFG_FILE);
  loadConfig();
  loadTable();
  applyChannel();

  if (fresh && _num_aps == 0) {
    startLearning((uint32_t)ROGUE_INITIAL_LEARN_MINS * 60);
  } else if (_cfg.learn_remaining_secs > 0) {
    startLearning(_cfg.learn_remaining_secs);   // resume interrupted learn period
  }

  if (_cfg.heartbeat_hours > 0) {
    _next_heartbeat_millis = millis() + (unsigned long)_cfg.heartbeat_hours * 3600000UL;
  }
  scheduleNextScan(15000);   // first scan shortly after boot

  char msg[ROGUE_MAX_TEXT + 1];
  if (_learning) {
    snprintf(msg, sizeof(msg), "online: LEARNING %lum, %d known APs",
             (_learn_end_millis - millis()) / 60000UL + 1, numKnown());
  } else {
    snprintf(msg, sizeof(msg), "online: monitoring, %d known, %d unacked", numKnown(), numReported());
  }
  queueText(msg);
}

/* ------------------------------ persistence ------------------------------ */

void RogueApScanner::loadConfig() {
  File f = _fs->open(ROGUE_CFG_FILE, "r");
  if (f) {
    RogueConfig tmp;
    memset(&tmp, 0, sizeof(tmp));
    int n = f.read((uint8_t *) &tmp, sizeof(tmp));
    f.close();
    if (n == sizeof(tmp) && tmp.magic == ROGUE_CFG_MAGIC) {
      _cfg = tmp;
      return;
    }
  }
  setDefaults();
}

void RogueApScanner::saveConfig() {
  File f = _fs->open(ROGUE_CFG_FILE, "w", true);
  if (f) {
    f.write((const uint8_t *) &_cfg, sizeof(_cfg));
    f.close();
  }
}

void RogueApScanner::loadTable() {
  _num_aps = 0;
  if (_aps == NULL) return;

  File f = _fs->open(ROGUE_TABLE_FILE, "r");
  if (!f) return;

  uint32_t magic = 0;
  uint16_t count = 0;
  if (f.read((uint8_t *) &magic, 4) == 4 && magic == ROGUE_TABLE_MAGIC && f.read((uint8_t *) &count, 2) == 2) {
    while (_num_aps < count && _num_aps < ROGUE_MAX_APS) {
      if (f.read((uint8_t *) &_aps[_num_aps], sizeof(RogueAp)) != sizeof(RogueAp)) break;
      _aps[_num_aps].ssid[32] = 0;
      _num_aps++;
    }
  }
  f.close();
}

void RogueApScanner::saveTable() {
  if (_aps == NULL) return;

  File f = _fs->open(ROGUE_TABLE_FILE, "w", true);
  if (f) {
    uint32_t magic = ROGUE_TABLE_MAGIC;
    uint16_t count = _num_aps;
    f.write((const uint8_t *) &magic, 4);
    f.write((const uint8_t *) &count, 2);
    f.write((const uint8_t *) _aps, sizeof(RogueAp) * _num_aps);
    f.close();
  }
  _table_dirty = false;
}

void RogueApScanner::applyChannel() {
  memset(&_channel, 0, sizeof(_channel));
  if (_cfg.channel_set) {
    memcpy(_channel.secret, _cfg.channel_secret, 16);   // AES-128 key, upper 16 bytes zero
    mesh::Utils::sha256(_channel.hash, sizeof(_channel.hash), _cfg.channel_secret, 16);
  }
}

/* ------------------------------ tables ------------------------------ */

int RogueApScanner::numKnown() const {
  int n = 0;
  for (int i = 0; i < _num_aps; i++) if (_aps[i].state == ROGUE_STATE_KNOWN) n++;
  return n;
}

int RogueApScanner::numReported() const {
  int n = 0;
  for (int i = 0; i < _num_aps; i++) if (_aps[i].state == ROGUE_STATE_REPORTED) n++;
  return n;
}

RogueAp* RogueApScanner::findAp(const uint8_t* bssid) {
  for (int i = 0; i < _num_aps; i++) {
    if (memcmp(_aps[i].bssid, bssid, 6) == 0) return &_aps[i];
  }
  return NULL;
}

RogueAp* RogueApScanner::addAp(const uint8_t* bssid, uint8_t state, const char* ssid, uint8_t channel, int8_t rssi, uint8_t auth, uint32_t now) {
  if (_aps == NULL || _num_aps >= ROGUE_MAX_APS) {
    if (!_table_full_reported) {
      _table_full_reported = true;
      char msg[ROGUE_MAX_TEXT + 1];
      snprintf(msg, sizeof(msg), "WARN: AP table full (%d), new APs not tracked. Use 'rogue forget'", ROGUE_MAX_APS);
      queueText(msg);
    }
    return NULL;
  }
  RogueAp* ap = &_aps[_num_aps++];
  memset(ap, 0, sizeof(*ap));
  memcpy(ap->bssid, bssid, 6);
  ap->state = state;
  StrHelper::strncpy(ap->ssid, ssid, sizeof(ap->ssid));
  ap->channel = channel;
  ap->rssi = rssi;
  ap->auth = auth;
  ap->first_seen = ap->last_seen = now;
  _table_dirty = true;
  return ap;
}

void RogueApScanner::removeAp(int idx) {
  _num_aps--;
  for (int i = idx; i < _num_aps; i++) _aps[i] = _aps[i + 1];
  _table_full_reported = false;
  _table_dirty = true;
}

RogueAp* RogueApScanner::findKnownBySsid(const char* ssid) {
  if (ssid[0] == 0) return NULL;
  for (int i = 0; i < _num_aps; i++) {
    if (_aps[i].state == ROGUE_STATE_KNOWN && strcmp(_aps[i].ssid, ssid) == 0) return &_aps[i];
  }
  return NULL;
}

bool RogueApScanner::isAllowedByPattern(const uint8_t* bssid, const char* ssid) const {
  char hex[13];
  macToHex(hex, bssid);
  for (int i = 0; i < ROGUE_MAX_PATTERNS; i++) {
    const char* p = _cfg.patterns[i];
    if (p[0] == 0) continue;
    if (strncmp(p, "mac:", 4) == 0) {
      if (globMatch(&p[4], hex)) return true;
    } else if (globMatch(p, ssid)) {
      return true;
    }
  }
  return false;
}

int RogueApScanner::bumpCandidate(const uint8_t* bssid) {
  for (int i = 0; i < _num_cand; i++) {
    if (memcmp(_cand[i].bssid, bssid, 6) == 0) {
      if (_cand[i].last_scan != _scan_seq) {
        _cand[i].hits++;
        _cand[i].last_scan = _scan_seq;
      }
      return _cand[i].hits;
    }
  }
  if (_num_cand >= ROGUE_MAX_CANDIDATES) {   // evict the stalest candidate
    int oldest = 0;
    for (int i = 1; i < _num_cand; i++) {
      if (_cand[i].last_scan < _cand[oldest].last_scan) oldest = i;
    }
    _cand[oldest] = _cand[--_num_cand];
  }
  auto c = &_cand[_num_cand++];
  memcpy(c->bssid, bssid, 6);
  c->hits = 1;
  c->last_scan = _scan_seq;
  return 1;
}

void RogueApScanner::removeCandidate(const uint8_t* bssid) {
  for (int i = 0; i < _num_cand; i++) {
    if (memcmp(_cand[i].bssid, bssid, 6) == 0) {
      _cand[i] = _cand[--_num_cand];
      return;
    }
  }
}

void RogueApScanner::expireCandidates() {
  uint32_t max_age = (uint32_t)_cfg.confirm_scans + 2;
  int i = 0;
  while (i < _num_cand) {
    if (_scan_seq - _cand[i].last_scan > max_age) {
      _cand[i] = _cand[--_num_cand];
    } else {
      i++;
    }
  }
}

/* ------------------------------ scanning ------------------------------ */

void RogueApScanner::startLearning(uint32_t secs) {
  _learning = true;
  _learn_end_millis = millis() + secs * 1000UL;
  _num_cand = 0;
  _cfg.learn_remaining_secs = secs;
  saveConfig();
  scheduleNextScan(2000);
}

void RogueApScanner::stopLearning(bool announce) {
  _learning = false;
  _cfg.learn_remaining_secs = 0;
  saveConfig();
  if (_table_dirty) saveTable();
  if (announce) {
    char msg[ROGUE_MAX_TEXT + 1];
    snprintf(msg, sizeof(msg), "LEARN done: %d APs in baseline, now monitoring", numKnown());
    queueText(msg);
  }
  scheduleNextScan((unsigned long)_cfg.interval_mins * 60000UL);
}

void RogueApScanner::scheduleNextScan(unsigned long delay_millis) {
  _next_scan_millis = millis() + delay_millis;
}

void RogueApScanner::startScan() {
  WiFi.mode(WIFI_STA);   // station mode, never associates. Passive scan = receive only

  // scan all 2.4GHz channels 1-13 (passive listening only, so no regulatory TX concerns)
  wifi_country_t country = { "01", 1, 13, 0, WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&country);

  int16_t rc = WiFi.scanNetworks(true /*async*/, true /*show_hidden*/, true /*passive*/, _cfg.dwell_ms);
  if (rc == WIFI_SCAN_FAILED) {
    _failed_scans++;
    WiFi.mode(WIFI_OFF);
    scheduleNextScan(60000);   // retry in a minute
    return;
  }
  _scanning = true;
  _scan_start_millis = millis();
}

void RogueApScanner::finishScan() {
  WiFi.scanDelete();
  if (!(WiFi.getMode() & WIFI_MODE_AP)) {   // don't kill an OTA soft-AP started mid-scan
    WiFi.mode(WIFI_OFF);   // radio off between scans, to save power
  }
  _scanning = false;
  if (_learning) {
    scheduleNextScan(ROGUE_LEARN_SCAN_SECS * 1000UL);
  } else {
    scheduleNextScan((unsigned long)_cfg.interval_mins * 60000UL);
  }
}

void RogueApScanner::processScan(int n) {
  uint32_t now = _rtc->getCurrentTime();
  _scan_seq++;
  _total_scans++;
  _last_scan_secs = now;
  _last_scan_count = n;
  _last_scan_new = 0;

  int alerts = 0, suppressed = 0;
  bool any_new = false, any_changed = false;
  char first_event[ROGUE_MAX_TEXT + 1];
  first_event[0] = 0;

  for (int i = 0; i < n; i++) {
    auto rec = (wifi_ap_record_t *) WiFi.getScanInfoByIndex(i);
    if (rec == NULL) continue;

    char ssid[33];
    memcpy(ssid, rec->ssid, 32);
    ssid[32] = 0;
    uint8_t channel = rec->primary;
    int8_t rssi = rec->rssi;
    uint8_t auth = rec->authmode;

    char msg[ROGUE_MAX_TEXT + 1];
    msg[0] = 0;

    RogueAp* ap = findAp(rec->bssid);
    if (_learning) {
      if (ap) {
        if (ap->state != ROGUE_STATE_KNOWN || strcmp(ap->ssid, ssid) != 0 || ap->auth != auth) _table_dirty = true;
        ap->state = ROGUE_STATE_KNOWN;
        StrHelper::strncpy(ap->ssid, ssid, sizeof(ap->ssid));
        ap->auth = auth;
      } else {
        ap = addAp(rec->bssid, ROGUE_STATE_KNOWN, ssid, channel, rssi, auth, now);
      }
    } else if (ap) {
      bool ssid_changed = strcmp(ap->ssid, ssid) != 0;
      bool auth_changed = ap->auth != auth;
      if (ap->state == ROGUE_STATE_KNOWN && (ssid_changed || auth_changed) && !isAllowedByPattern(rec->bssid, ssid)) {
        char mac[18], old_s[33], new_s[33];
        macToStr(mac, rec->bssid);
        ssidForDisplay(old_s, sizeof(old_s), ap->ssid);
        ssidForDisplay(new_s, sizeof(new_s), ssid);
        if (ssid_changed) {
          snprintf(msg, sizeof(msg), "CHANGED AP %s \"%s\"->\"%s\" %s->%s ch%d %ddBm",
                   mac, old_s, new_s, authName(ap->auth), authName(auth), channel, rssi);
        } else {
          snprintf(msg, sizeof(msg), "CHANGED AP %s \"%s\" security %s->%s ch%d %ddBm",
                   mac, new_s, authName(ap->auth), authName(auth), channel, rssi);
        }
        ap->state = ROGUE_STATE_REPORTED;   // needs re-approval via 'rogue allow'
        any_changed = true;
      }
      if (ssid_changed || auth_changed) _table_dirty = true;
      StrHelper::strncpy(ap->ssid, ssid, sizeof(ap->ssid));
      ap->auth = auth;
    } else if (!isAllowedByPattern(rec->bssid, ssid) && (_cfg.min_rssi == 0 || rssi >= _cfg.min_rssi)) {
      if (bumpCandidate(rec->bssid) >= _cfg.confirm_scans) {
        removeCandidate(rec->bssid);
        ap = addAp(rec->bssid, ROGUE_STATE_REPORTED, ssid, channel, rssi, auth, now);
        if (ap) {
          char mac[18], disp[33];
          macToStr(mac, rec->bssid);
          ssidForDisplay(disp, sizeof(disp), ssid);
          snprintf(msg, sizeof(msg), "NEW AP \"%s\" %s ch%d %ddBm %s%s", disp, mac, channel, rssi, authName(auth),
                   findKnownBySsid(ssid) ? " [same SSID as a known AP!]" : "");
          any_new = true;
        }
      }
    }

    if (ap) {
      ap->last_seen = now;
      ap->rssi = rssi;
      ap->channel = channel;
    }

    if (msg[0]) {
      _last_scan_new++;
      if (first_event[0] == 0) StrHelper::strncpy(first_event, msg, sizeof(first_event));
      if (alerts < _cfg.max_alerts_per_scan) {
        queueText(msg);
        alerts++;
      } else {
        suppressed++;
      }
    }
  }

  if (any_new) notifyMonitorEvent(MON_EV_NEW_AP);
  if (any_changed) notifyMonitorEvent(MON_EV_CHANGED_AP);

  if (suppressed > 0) {
    char msg[ROGUE_MAX_TEXT + 1];
    snprintf(msg, sizeof(msg), "+%d more new/changed APs (see 'rogue new')", suppressed);
    queueText(msg);
  }
  if (_last_scan_new > 0) {
    char alert[ROGUE_MAX_TEXT + 1];
    if (_last_scan_new == 1) {
      snprintf(alert, sizeof(alert), "ROGUE: %s", first_event);
    } else {
      snprintf(alert, sizeof(alert), "ROGUE: %d new/changed APs. %s", _last_scan_new, first_event);
    }
    queueAlert(alert);
  }

  expireCandidates();

  if (_learning) {
    long remaining = (long)(_learn_end_millis - millis()) / 1000;
    _cfg.learn_remaining_secs = remaining > 0 ? remaining : 1;
    saveConfig();
  }
  if (_table_dirty) saveTable();
}

void RogueApScanner::loop() {
  if (_aps == NULL) return;

  if (_learning && (long)(millis() - _learn_end_millis) >= 0 && !_scanning) {
    stopLearning(true);
  }

  if (_scanning) {
    int16_t n = WiFi.scanComplete();
    if (n >= 0) {
      processScan(n);
      finishScan();
    } else if (n == WIFI_SCAN_FAILED || millis() - _scan_start_millis > SCAN_TIMEOUT_MILLIS) {
      _failed_scans++;
      finishScan();
    }
  } else if ((long)(millis() - _next_scan_millis) >= 0) {
    if (WiFi.getMode() & WIFI_MODE_AP) {   // OTA update in progress ('start ota') - leave WiFi alone
      _ota_paused = true;
      scheduleNextScan(60000);
    } else {
      _ota_paused = false;
      startScan();
    }
  }

  if (_cfg.heartbeat_hours > 0 && _next_heartbeat_millis && (long)(millis() - _next_heartbeat_millis) >= 0) {
    _next_heartbeat_millis = millis() + (unsigned long)_cfg.heartbeat_hours * 3600000UL;
    char msg[ROGUE_MAX_TEXT + 1];
    snprintf(msg, sizeof(msg), "OK: %s, %d known, %d unacked, last scan %d APs, %lu scans (%lu failed)",
             _learning ? "learning" : "monitoring", numKnown(), numReported(), _last_scan_count,
             (unsigned long)_total_scans, (unsigned long)_failed_scans);
    queueText(msg);
  }
}

/* ------------------------------ outbound queue ------------------------------ */

void RogueApScanner::queueText(const char* text) {
  if (_tx_count >= ROGUE_TXQ_SIZE) {
    _tx_dropped++;
    return;
  }
  unsigned long due = millis() + random(500, 3000);   // a little jitter
  if (_tx_count > 0) {
    auto prev = &_txq[(_tx_head + _tx_count - 1) % ROGUE_TXQ_SIZE];
    unsigned long min_due = prev->due_millis + (unsigned long)_cfg.tx_gap_secs * 1000UL;
    if ((long)(min_due - due) > 0) due = min_due;
  }
  auto m = &_txq[(_tx_head + _tx_count) % ROGUE_TXQ_SIZE];
  m->due_millis = due;
  StrHelper::strncpy(m->text, text, sizeof(m->text));
  _tx_count++;
}

bool RogueApScanner::popChannelText(char* dest) {
  if (_tx_count == 0) return false;
  auto m = &_txq[_tx_head];
  if ((long)(millis() - m->due_millis) < 0) return false;

  strcpy(dest, m->text);
  _tx_head = (_tx_head + 1) % ROGUE_TXQ_SIZE;
  _tx_count--;
  return true;
}

void RogueApScanner::queueAlert(const char* text) {
  if (!_cfg.dm_alerts) return;
  if (_alert_count >= ROGUE_ALERTQ_SIZE) {   // drop oldest, newest is more relevant
    _alert_head = (_alert_head + 1) % ROGUE_ALERTQ_SIZE;
    _alert_count--;
  }
  StrHelper::strncpy(_alerts[(_alert_head + _alert_count) % ROGUE_ALERTQ_SIZE], text, ROGUE_MAX_TEXT + 1);
  _alert_count++;
}

bool RogueApScanner::popAlertText(char* dest) {
  if (_alert_count == 0) return false;
  strcpy(dest, _alerts[_alert_head]);
  _alert_head = (_alert_head + 1) % ROGUE_ALERTQ_SIZE;
  _alert_count--;
  return true;
}

/* ------------------------------ CLI ------------------------------ */

void RogueApScanner::formatEntry(char* dest, int dest_size, const RogueAp& ap) {
  char hex[13], disp[17];
  macToHex(hex, ap.bssid);
  ssidForDisplay(disp, sizeof(disp), ap.ssid);   // truncated to 16 chars, to fit pages
  snprintf(dest, dest_size, "%s %s c%d %d %s", hex, disp, ap.channel, ap.rssi, authName(ap.auth));
}

void RogueApScanner::formatStatus(char* reply) {
  char mode[24];
  if (_ota_paused) {
    strcpy(mode, "PAUSED(ota)");
  } else if (_learning) {
    sprintf(mode, "LEARN %lum left", (unsigned long)((_learn_end_millis - millis()) / 60000UL + 1));
  } else {
    strcpy(mode, "MONITOR");
  }
  uint32_t now = _rtc->getCurrentTime();
  snprintf(reply, REPLY_MAX, "%s known:%d new:%d cand:%d last:%d APs %lus ago scans:%lu fail:%lu ch:%s",
           mode, numKnown(), numReported(), _num_cand, _last_scan_count,
           (unsigned long)(_last_scan_secs ? now - _last_scan_secs : 0),
           (unsigned long)_total_scans, (unsigned long)_failed_scans,
           _cfg.channel_set ? _cfg.channel_name : "(none)");
}

void RogueApScanner::formatList(char* reply, uint8_t state_mask, int page) {
  int total = 0;
  for (int i = 0; i < _num_aps; i++) if (_aps[i].state & state_mask) total++;
  int pages = (total + LIST_PAGE_SIZE - 1) / LIST_PAGE_SIZE;
  if (total == 0) {
    strcpy(reply, "(none)");
    return;
  }
  if (page < 1) page = 1;
  if (page > pages) page = pages;

  int len = sprintf(reply, "p%d/%d (%d)", page, pages, total);
  int skip = (page - 1) * LIST_PAGE_SIZE, shown = 0;
  for (int i = 0; i < _num_aps && shown < LIST_PAGE_SIZE; i++) {
    if (!(_aps[i].state & state_mask)) continue;
    if (skip > 0) { skip--; continue; }
    char line[64];
    formatEntry(line, sizeof(line), _aps[i]);
    len += snprintf(&reply[len], REPLY_MAX - len, "\n%s%s", line,
                    (state_mask != ROGUE_STATE_REPORTED && _aps[i].state == ROGUE_STATE_REPORTED) ? " *" : "");
    if (len >= REPLY_MAX) { reply[REPLY_MAX - 1] = 0; break; }
    shown++;
  }
}

// match entries by hex BSSID prefix (min 4 digits), or "all". Returns number of matches
int RogueApScanner::resolveAps(const char* arg, uint8_t state_mask, int* idx_out) {
  if (strcmp(arg, "all") == 0) {
    int n = 0;
    for (int i = 0; i < _num_aps; i++) if (_aps[i].state & state_mask) n++;
    *idx_out = -1;
    return n;
  }
  char want[13];
  int digits = normaliseHex(want, sizeof(want), arg);
  if (digits < 4) return 0;

  int n = 0;
  for (int i = 0; i < _num_aps; i++) {
    if (!(_aps[i].state & state_mask)) continue;
    char hex[13];
    macToHex(hex, _aps[i].bssid);
    if (strncmp(hex, want, digits) == 0) {
      *idx_out = i;
      n++;
    }
  }
  return n;
}

bool RogueApScanner::handleCommand(char* command, char* reply) {
  if (strcmp(command, "rogue") == 0) {
    formatStatus(reply);
    return true;
  }
  if (memcmp(command, "rogue ", 6) != 0) return false;
  if (_aps == NULL) {
    strcpy(reply, "Err - out of memory");
    return true;
  }

  char* cmd = skipSpaces(&command[6]);
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
    strcpy(reply, "rogue status|mem|now|new [p]|list [p]|show <mac>|allow <mac|all>|forget <mac|all>|learn <m|stop>|config|reset");

  } else if (strcmp(cmd, "mem") == 0) {
    snprintf(reply, REPLY_MAX, "heap free:%lu min:%lu largest:%lu scans:%lu",
             (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
             (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)_total_scans);

  } else if (strcmp(cmd, "config") == 0) {
    snprintf(reply, REPLY_MAX, "interval:%dm dwell:%dms rssi:%d confirm:%d maxalerts:%d heartbeat:%dh gap:%ds dm:%s ch:%s",
             _cfg.interval_mins, _cfg.dwell_ms, _cfg.min_rssi, _cfg.confirm_scans, _cfg.max_alerts_per_scan,
             _cfg.heartbeat_hours, _cfg.tx_gap_secs, _cfg.dm_alerts ? "on" : "off",
             _cfg.channel_set ? _cfg.channel_name : "(none)");

  } else if (strcmp(cmd, "channel") == 0) {
    if (*arg == 0) {
      if (_cfg.channel_set) {
        char hex[33];
        mesh::Utils::toHex(hex, _cfg.channel_secret, 16);
        snprintf(reply, REPLY_MAX, "%s key:%s", _cfg.channel_name, hex);
      } else {
        strcpy(reply, "(none)");
      }
    } else if (strcmp(arg, "off") == 0) {
      _cfg.channel_set = 0;
      memset(_cfg.channel_secret, 0, sizeof(_cfg.channel_secret));
      _cfg.channel_name[0] = 0;
      saveConfig();
      applyChannel();
      strcpy(reply, "OK - channel reports disabled");
    } else {
      char* key = strchr(arg, ' ');
      if (key) { *key = 0; key = skipSpaces(key + 1); }

      if (arg[0] == '#' && (key == NULL || *key == 0)) {   // hashtag channel: key = sha256(name)[0..15]
        if (strlen(arg) < 2 || strlen(arg) >= sizeof(_cfg.channel_name)) {
          strcpy(reply, "Err - bad channel name");
          return true;
        }
        mesh::Utils::sha256(_cfg.channel_secret, 16, (const uint8_t *) arg, strlen(arg));
      } else if (key && *key) {
        if (strlen(arg) >= sizeof(_cfg.channel_name) || !mesh::Utils::fromHex(_cfg.channel_secret, 16, key)) {
          strcpy(reply, "Err - need: rogue channel <name> <32 hex key>");
          return true;
        }
      } else {
        strcpy(reply, "Err - need: rogue channel #name  OR  rogue channel <name> <32 hex key>");
        return true;
      }
      StrHelper::strncpy(_cfg.channel_name, arg, sizeof(_cfg.channel_name));
      _cfg.channel_set = 1;
      saveConfig();
      applyChannel();
      char hex[33];
      mesh::Utils::toHex(hex, _cfg.channel_secret, 16);
      snprintf(reply, REPLY_MAX, "OK - %s key:%s", _cfg.channel_name, hex);
    }

  } else if (strcmp(cmd, "interval") == 0) {
    int v = atoi(arg);
    if (v < 1 || v > 1440) {
      strcpy(reply, "Err - 1..1440 mins");
    } else {
      _cfg.interval_mins = v;
      saveConfig();
      if (!_learning && !_scanning) scheduleNextScan((unsigned long)v * 60000UL);
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "dwell") == 0) {
    int v = atoi(arg);
    if (v < 100 || v > 1500) {
      strcpy(reply, "Err - 100..1500 ms");
    } else {
      _cfg.dwell_ms = v;
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

  } else if (strcmp(cmd, "confirm") == 0) {
    int v = atoi(arg);
    if (v < 1 || v > 10) {
      strcpy(reply, "Err - 1..10 scans");
    } else {
      _cfg.confirm_scans = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "maxalerts") == 0) {
    int v = atoi(arg);
    if (v < 1 || v > ROGUE_TXQ_SIZE / 2) {
      sprintf(reply, "Err - 1..%d", ROGUE_TXQ_SIZE / 2);
    } else {
      _cfg.max_alerts_per_scan = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "heartbeat") == 0) {
    int v = atoi(arg);
    if (v < 0 || v > 168) {
      strcpy(reply, "Err - 0..168 hours (0 = off)");
    } else {
      _cfg.heartbeat_hours = v;
      saveConfig();
      _next_heartbeat_millis = v > 0 ? millis() + (unsigned long)v * 3600000UL : 0;
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "gap") == 0) {
    int v = atoi(arg);
    if (v < 1 || v > 120) {
      strcpy(reply, "Err - 1..120 secs");
    } else {
      _cfg.tx_gap_secs = v;
      saveConfig();
      strcpy(reply, "OK");
    }

  } else if (strcmp(cmd, "dm") == 0) {
    if (strcmp(arg, "on") == 0 || strcmp(arg, "off") == 0) {
      _cfg.dm_alerts = strcmp(arg, "on") == 0;
      saveConfig();
      strcpy(reply, "OK");
    } else {
      sprintf(reply, "dm alerts: %s", _cfg.dm_alerts ? "on" : "off");
    }

  } else if (strcmp(cmd, "learn") == 0) {
    if (strcmp(arg, "stop") == 0) {
      if (_learning) {
        stopLearning(false);
        sprintf(reply, "OK - monitoring, %d known", numKnown());
      } else {
        strcpy(reply, "Err - not learning");
      }
    } else {
      int v = atoi(arg);
      if (v < 1 || v > 10080) {
        strcpy(reply, "Err - rogue learn <1..10080 mins> | stop");
      } else {
        startLearning((uint32_t)v * 60);
        sprintf(reply, "OK - learning for %d mins", v);
      }
    }

  } else if (strcmp(cmd, "now") == 0) {
    if (_scanning) {
      strcpy(reply, "Err - scan in progress");
    } else {
      scheduleNextScan(0);
      strcpy(reply, "OK - scanning");
    }

  } else if (strcmp(cmd, "new") == 0) {
    formatList(reply, ROGUE_STATE_REPORTED, atoi(arg));

  } else if (strcmp(cmd, "list") == 0) {
    formatList(reply, ROGUE_STATE_KNOWN | ROGUE_STATE_REPORTED, atoi(arg));

  } else if (strcmp(cmd, "show") == 0) {
    int idx;
    int n = resolveAps(arg, ROGUE_STATE_KNOWN | ROGUE_STATE_REPORTED, &idx);
    if (n != 1 || idx < 0) {
      strcpy(reply, n > 1 ? "Err - ambiguous, give more of the BSSID" : "Err - not found");
    } else {
      auto ap = &_aps[idx];
      char mac[18], disp[33];
      macToStr(mac, ap->bssid);
      ssidForDisplay(disp, sizeof(disp), ap->ssid);
      uint32_t now = _rtc->getCurrentTime();
      snprintf(reply, REPLY_MAX, "%s \"%s\" %s ch%d %ddBm %s first:%lum ago last:%lum ago",
               mac, disp, ap->state == ROGUE_STATE_KNOWN ? "KNOWN" : "NEW", ap->channel, ap->rssi, authName(ap->auth),
               (unsigned long)((now - ap->first_seen) / 60), (unsigned long)((now - ap->last_seen) / 60));
    }

  } else if (strcmp(cmd, "allow") == 0) {
    int idx;
    int n = resolveAps(arg, ROGUE_STATE_REPORTED, &idx);
    if (n == 0) {
      strcpy(reply, "Err - no matching new AP");
    } else if (n > 1 && idx >= 0) {
      strcpy(reply, "Err - ambiguous, give more of the BSSID");
    } else {
      for (int i = 0; i < _num_aps; i++) {
        if (_aps[i].state == ROGUE_STATE_REPORTED && (idx < 0 || i == idx)) _aps[i].state = ROGUE_STATE_KNOWN;
      }
      saveTable();
      sprintf(reply, "OK - %d added to baseline", n);
    }

  } else if (strcmp(cmd, "forget") == 0) {
    int idx;
    int n = resolveAps(arg, ROGUE_STATE_KNOWN | ROGUE_STATE_REPORTED, &idx);
    if (n == 0) {
      strcpy(reply, "Err - not found");
    } else if (n > 1 && idx >= 0) {
      strcpy(reply, "Err - ambiguous, give more of the BSSID");
    } else if (idx < 0) {
      strcpy(reply, "Err - use 'rogue reset confirm' to forget all");
    } else {
      removeAp(idx);
      saveTable();
      strcpy(reply, "OK - forgotten (will alert if seen again)");
    }

  } else if (strcmp(cmd, "pattern") == 0) {
    if (memcmp(arg, "add ", 4) == 0) {
      char* p = skipSpaces(&arg[4]);
      int slot = -1;
      for (int i = 0; i < ROGUE_MAX_PATTERNS; i++) if (_cfg.patterns[i][0] == 0) { slot = i; break; }
      if (*p == 0 || strlen(p) > ROGUE_PATTERN_LEN) {
        sprintf(reply, "Err - pattern 1..%d chars", ROGUE_PATTERN_LEN);
      } else if (slot < 0) {
        strcpy(reply, "Err - pattern table full");
      } else {
        StrHelper::strncpy(_cfg.patterns[slot], p, sizeof(_cfg.patterns[slot]));
        saveConfig();
        sprintf(reply, "OK - pattern %d", slot + 1);
      }
    } else if (memcmp(arg, "del ", 4) == 0) {
      int v = atoi(&arg[4]);
      if (v < 1 || v > ROGUE_MAX_PATTERNS || _cfg.patterns[v - 1][0] == 0) {
        strcpy(reply, "Err - bad pattern number");
      } else {
        _cfg.patterns[v - 1][0] = 0;
        saveConfig();
        strcpy(reply, "OK");
      }
    } else {
      int len = 0;
      reply[0] = 0;
      for (int i = 0; i < ROGUE_MAX_PATTERNS && len < REPLY_MAX; i++) {
        if (_cfg.patterns[i][0] == 0) continue;
        len += snprintf(&reply[len], REPLY_MAX - len, "%s%d:%s", len ? "\n" : "", i + 1, _cfg.patterns[i]);
      }
      if (len == 0) strcpy(reply, "(no patterns) rogue pattern add <ssid glob>|mac:<hex glob>");
    }

  } else if (strcmp(cmd, "test") == 0) {
    if (!_cfg.channel_set) {
      strcpy(reply, "Err - no channel set");
    } else {
      queueText("TEST message from rogue AP monitor");
      strcpy(reply, "OK - queued");
    }

  } else if (strcmp(cmd, "reset") == 0) {
    if (strcmp(arg, "confirm") == 0) {
      _num_aps = 0;
      _num_cand = 0;
      _table_full_reported = false;
      saveTable();
      strcpy(reply, "OK - baseline cleared. Use 'rogue learn <mins>'");
    } else {
      strcpy(reply, "Clears ALL known/new APs. Send 'rogue reset confirm'");
    }

  } else {
    strcpy(reply, "Err - unknown. Try 'rogue help'");
  }
  return true;
}
