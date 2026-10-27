#pragma once

// events raised by the rogue AP scanner and beacon tracker, for physical reactions (see PikaToy)
enum MonitorEvent {
  MON_EV_NEW_AP = 0,        // one or more new APs reported in a scan
  MON_EV_CHANGED_AP,        // a known AP changed SSID / security
  MON_EV_BEACON_NEW,        // beacon first heard
  MON_EV_BEACON_MISSING,    // beacon not heard within timeout
  MON_EV_BEACON_BACK,       // missing beacon heard again
  MON_EV_COUNT
};

void notifyMonitorEvent(MonitorEvent ev);
