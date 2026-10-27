#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <helpers/IdentityStore.h>   // for FILESYSTEM

/*
 * Passive rogue access point detector.
 *
 *  - periodically runs a passive WiFi scan (listen-only, no probe requests are transmitted)
 *  - in LEARN mode, every BSSID heard is added to the baseline (KNOWN)
 *  - in MONITOR mode, a BSSID not in the baseline (and not matching an allow pattern) becomes a
 *    candidate, and is REPORTED once it has been heard in 'confirm' scans
 *  - SSID / security changes on a known BSSID are also reported
 *  - reports are queued as text lines, for sending on a group channel (and optionally as DM alerts)
 */

#ifndef ROGUE_MAX_APS
  #define ROGUE_MAX_APS          512
#endif
#ifndef ROGUE_MAX_CANDIDATES
  #define ROGUE_MAX_CANDIDATES   64
#endif
#ifndef ROGUE_INITIAL_LEARN_MINS
  #define ROGUE_INITIAL_LEARN_MINS   60     // learn period on a fresh install
#endif
#define ROGUE_MAX_PATTERNS       8
#define ROGUE_PATTERN_LEN        32
#define ROGUE_TXQ_SIZE           24
#define ROGUE_ALERTQ_SIZE        4
#define ROGUE_MAX_TEXT           128      // channel message body (node name prefix is added on top)
#define ROGUE_LEARN_SCAN_SECS    60       // scan interval while learning

#define ROGUE_STATE_KNOWN        1
#define ROGUE_STATE_REPORTED     2

struct RogueAp {
  uint8_t  bssid[6];
  uint8_t  state;          // ROGUE_STATE_*
  uint8_t  channel;
  int8_t   rssi;           // most recent
  uint8_t  auth;           // wifi_auth_mode_t
  char     ssid[33];       // empty = hidden
  uint32_t first_seen;     // RTC secs
  uint32_t last_seen;      // RTC secs
};

struct RogueCandidate {
  uint8_t  bssid[6];
  uint8_t  hits;
  uint32_t last_scan;      // scan sequence number when last heard
};

struct RogueConfig {
  uint32_t magic;
  char     channel_name[32];
  uint8_t  channel_secret[16];   // AES-128 channel key
  uint8_t  channel_set;
  uint8_t  dm_alerts;            // also send DM alerts to admins (ACL alert perms)
  uint8_t  confirm_scans;        // scans a new BSSID must be heard in, before reporting
  uint8_t  max_alerts_per_scan;
  int8_t   min_rssi;             // ignore new APs weaker than this (0 = no filter)
  uint16_t interval_mins;        // monitor scan interval
  uint16_t dwell_ms;             // passive listen time per WiFi channel
  uint16_t heartbeat_hours;      // 0 = off
  uint16_t tx_gap_secs;          // spacing between channel messages
  uint32_t learn_remaining_secs; // persisted learn progress (survives reboot)
  char     patterns[ROGUE_MAX_PATTERNS][ROGUE_PATTERN_LEN + 1];   // SSID globs, or "mac:" + hex BSSID glob
};

struct RogueTxMsg {
  uint32_t due_millis;
  char     text[ROGUE_MAX_TEXT + 1];
};

class RogueApScanner {
public:
  RogueApScanner();

  void begin(FILESYSTEM* fs, mesh::RTCClock* rtc);
  void loop();

  // CLI: handles "rogue ..." commands, returns false if not a rogue command
  bool handleCommand(char* command, char* reply);

  // next channel message that is due for sending
  bool popChannelText(char* dest);
  // DM alerts for admins (if dm alerts enabled)
  void queueAlert(const char* text);
  bool hasAlertText() const { return _alert_count > 0; }
  bool popAlertText(char* dest);

  bool hasChannel() const { return _cfg.channel_set != 0; }
  const mesh::GroupChannel& getChannel() const { return _channel; }

  bool isLearning() const { return _learning; }
  int  numKnown() const;
  int  numReported() const;
  int  lastScanCount() const { return _last_scan_count; }
  void queueText(const char* text);

private:
  FILESYSTEM* _fs;
  mesh::RTCClock* _rtc;
  RogueConfig _cfg;
  mesh::GroupChannel _channel;

  RogueAp* _aps;
  int _num_aps;
  RogueCandidate _cand[ROGUE_MAX_CANDIDATES];
  int _num_cand;

  bool _learning;
  unsigned long _learn_end_millis;
  bool _scanning;
  unsigned long _scan_start_millis, _next_scan_millis, _next_heartbeat_millis;
  uint32_t _scan_seq, _last_scan_secs;
  int _last_scan_count, _last_scan_new;
  uint32_t _total_scans, _failed_scans;
  bool _table_dirty, _table_full_reported, _ota_paused;

  RogueTxMsg _txq[ROGUE_TXQ_SIZE];
  int _tx_head, _tx_count;
  uint32_t _tx_dropped;
  char _alerts[ROGUE_ALERTQ_SIZE][ROGUE_MAX_TEXT + 1];
  int _alert_head, _alert_count;

  void setDefaults();
  void loadConfig();
  void saveConfig();
  void loadTable();
  void saveTable();
  void applyChannel();

  void startLearning(uint32_t secs);
  void stopLearning(bool announce);
  void scheduleNextScan(unsigned long delay_millis);
  void startScan();
  void processScan(int n);
  void finishScan();

  RogueAp* findAp(const uint8_t* bssid);
  RogueAp* addAp(const uint8_t* bssid, uint8_t state, const char* ssid, uint8_t channel, int8_t rssi, uint8_t auth, uint32_t now);
  void removeAp(int idx);
  RogueAp* findKnownBySsid(const char* ssid);
  bool isAllowedByPattern(const uint8_t* bssid, const char* ssid) const;
  int  bumpCandidate(const uint8_t* bssid);
  void removeCandidate(const uint8_t* bssid);
  void expireCandidates();

  int  resolveAps(const char* arg, uint8_t state_mask, int* idx_out);
  void formatStatus(char* reply);
  void formatList(char* reply, uint8_t state_mask, int page);
  void formatEntry(char* dest, int dest_size, const RogueAp& ap);
};

extern RogueApScanner rogue;
