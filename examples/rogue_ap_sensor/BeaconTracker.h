#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <helpers/IdentityStore.h>   // for FILESYSTEM

/*
 * BLE beacon presence tracker.
 *
 *  - continuously scans BLE advertisements (shares the 2.4GHz radio with WiFi via coexistence)
 *  - any advertiser whose name starts with the configured prefix (default "RF") is tracked
 *  - reports when a beacon is FIRST heard, when it goes MISSING (not heard for 'timeout' mins),
 *    and when a missing beacon is heard AGAIN
 *  - tracked beacons persist across reboots (a reboot restarts each beacon's missing timer)
 */

#ifndef BEACON_MAX
  #define BEACON_MAX           64
#endif
#define BEACON_NAME_LEN        20
#define BEACON_PREFIX_LEN      31     // comma-separated list of name prefixes, eg. "RF,TG"
#define BEACON_MAX_TEXT        128

#define BEACON_STATE_PRESENT   1
#define BEACON_STATE_MISSING   2

#define BEACON_FLAG_NEW        0x01   // first heard, report pending
#define BEACON_FLAG_BACK       0x02   // heard again after missing, report pending

struct BeaconEntry {
  uint8_t  mac[6];
  uint8_t  addr_type;
  uint8_t  state;          // BEACON_STATE_*
  int8_t   rssi;           // most recent
  uint8_t  flags;          // BEACON_FLAG_* (runtime)
  char     name[BEACON_NAME_LEN + 1];
  uint32_t first_seen;     // RTC secs
  uint32_t last_seen;      // RTC secs (refreshed from last_seen_ms when processed)
  uint32_t sightings;
  uint32_t last_seen_ms;   // millis() (runtime)
};

struct BeaconConfig {
  uint32_t magic;
  char     prefix[BEACON_PREFIX_LEN + 1];
  uint16_t timeout_mins;   // missing after this long without a sighting
  int8_t   min_rssi;       // ignore NEW beacons weaker than this (0 = off)
  uint8_t  enabled;
  uint8_t  active_scan;    // request scan responses (needed if the name is only in the scan response)
  uint8_t  key_by_name;    // match by name instead of MAC (for beacons with rotating addresses)
  uint8_t  duty_pct;       // BLE listen duty cycle, 5..100 (0 = default)
  uint8_t  reserved;
};

class RogueApScanner;

class BeaconTracker {
public:
  BeaconTracker();

  void begin(FILESYSTEM* fs, mesh::RTCClock* rtc, RogueApScanner* out);
  void loop();

  // CLI: handles "beacon ..." commands, returns false if not a beacon command
  bool handleCommand(char* command, char* reply);

  void getCounts(int& present, int& missing);

  // called from the BT task
  void onAdvert(const uint8_t* mac, uint8_t addr_type, int rssi, const char* name);

private:
  FILESYSTEM* _fs;
  mesh::RTCClock* _rtc;
  RogueApScanner* _out;
  BeaconConfig _cfg;

  portMUX_TYPE _lock;
  BeaconEntry _beacons[BEACON_MAX];
  int _num_beacons;
  volatile uint32_t _adverts_seen;
  volatile bool _table_full, _table_changed;
  bool _table_full_reported;

  bool _ble_ready, _scanning;
  unsigned long _next_check_millis, _next_restart_millis, _scan_started_millis;

  void setDefaults();
  void loadConfig();
  void saveConfig();
  void loadTable();
  void saveTable();

  void startScan();
  void stopScan();

  bool nameMatchesLocked(const char* name) const;
  int  findLocked(const uint8_t* mac, const char* name) const;
  void checkBeacons();
  void report(const char* text, bool alert);

  int  resolve(const char* arg, int* idx_out);
  void formatStatus(char* reply);
  void formatList(char* reply, uint8_t state_mask, int page);
};

extern BeaconTracker beacons;
