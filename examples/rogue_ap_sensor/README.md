# Rogue AP Monitor (sensor role)

Passive rogue Wi-Fi access point detector for ESP32 MeshCore boards (built for the Heltec V4).
It learns the 2.4 GHz access points that are normally present. After that, it reports any
**new** BSSID to an AES-128 group channel, and optionally as direct-message alerts to admins.
You administer it over the mesh with the standard remote CLI.

It is built on the `simple_sensor` role, so login, ACL, `password`, `clock sync`, `start ota`,
radio settings, telemetry and battery alerts all work as they do on a stock sensor.

## Build

| Board            | Env                      |
|------------------|--------------------------|
| Heltec V4 (OLED) | `heltec_v4_rogue_ap`     |
| Heltec V4 (TFT)  | `heltec_v4_tft_rogue_ap` |
| Heltec V2        | `Heltec_v2_rogue_ap` (AP table 256, no PSRAM) |

```
pio run -e heltec_v4_rogue_ap -t upload
```

Any other ESP32 variant can use it: copy the env, add `-I examples/simple_sensor`, and use the same `build_src_filter` lines.

## How it works

1. **Scan.** It runs a passive scan of channels 1–13. It only listens for beacons and never
   transmits probe requests. Wi-Fi is switched off between scans.
2. **Learn.** Every BSSID heard during a learn period goes into the baseline (KNOWN).
   - A fresh install learns for 60 minutes automatically (`ROGUE_INITIAL_LEARN_MINS`).
   - While learning, it scans every 60 s. A learn period carries on after a reboot.
3. **Monitor.** It scans every `interval` minutes. An unknown BSSID becomes a candidate. It is
   **reported** once it has been heard in `confirm` scans and is at least as strong as `rssi`.
   - Each BSSID is reported once only.
   - Reported BSSIDs are saved, so a reboot does not cause repeat alerts.
4. **Changes.** A known BSSID that changes its SSID or security mode is reported as `CHANGED` and
   goes back to NEW until you allow it again.
5. **Evil-twin hint.** If a new BSSID uses the SSID of a known AP, the alert gets
   `[same SSID as a known AP!]` added to it.

Example channel messages (the sender is the node name):

```
Rogue AP Monitor: NEW AP "DIRECT-7F-HP LaserJet" 3A:4B:5C:6D:7E:9F ch6 -61dBm WPA2
Rogue AP Monitor: CHANGED AP 3A:4B:5C:6D:7E:9F "Corp"->"Corp-Guest" WPA2->OPEN ch1 -55dBm
Rogue AP Monitor: OK: monitoring, 37 known, 1 unacked, last scan 39 APs, 288 scans (0 failed)
```

## Setup

1. Flash the firmware and add the node in the MeshCore app. Log in with the admin password
   (`password` by default).
2. **Change the admin password first:** `password <new>`.
3. Sync the clock from the app (`clock sync`), so "first/last seen" times make sense.
4. Set the report channel. Use the same key as a channel set up in your companion app:
   - `rogue channel #rogue-alerts` uses a hashtag channel (key = first 16 bytes of `sha256("#rogue-alerts")`).
   - `rogue channel Alerts 0123456789abcdef0123456789abcdef` uses a private channel with a 32-hex-digit (AES-128) key.
   - The reply echoes the key back. Compare it with the key the app shows for that channel.
5. Send `rogue test`. You should see the message on that channel.
6. Let it learn (`rogue learn 1440` for a day is a good baseline), then check `rogue status`.

## CLI reference

All commands work over the serial console and over the remote admin CLI. Replies are kept under
about 150 characters, so lists are paged 3 entries at a time. Anywhere a `<mac>` is expected you
can give a hex prefix of the BSSID (at least 4 digits; `:` and `-` are ignored), or `all`.

| Command | Description |
|---|---|
| `rogue` / `rogue status` | Mode, known/new/candidate counts, last scan |
| `rogue config` | Show all settings |
| `rogue help` | Command summary |
| `rogue channel` | Show the report channel and key |
| `rogue channel #name` | Use a hashtag channel |
| `rogue channel <name> <32hex>` | Use a private AES-128 channel |
| `rogue channel off` | Stop channel reports |
| `rogue test` | Send a test message to the channel |
| `rogue learn <mins>` | Learn for N minutes (1–10080). Adds everything heard to the baseline |
| `rogue learn stop` | End learning now |
| `rogue now` | Scan right away |
| `rogue new [page]` | List reported (un-acked) APs |
| `rogue list [page]` | List all tracked APs (`*` = un-acked) |
| `rogue show <mac>` | Details for one AP |
| `rogue allow <mac\|all>` | Move reported AP(s) into the baseline |
| `rogue forget <mac>` | Remove an AP (it will alert again if seen) |
| `rogue reset confirm` | Clear the whole table (then run `rogue learn`) |
| `rogue pattern` | List allow patterns |
| `rogue pattern add <glob>` | Never alert for SSIDs that match, e.g. `DIRECT-*-HP*`. `*` and `?` work and case is ignored |
| `rogue pattern add mac:<glob>` | Never alert for BSSIDs that match, e.g. `mac:F4CE36*` (vendor OUI) |
| `rogue pattern del <n>` | Remove pattern n |
| `rogue interval <mins>` | Scan interval while monitoring (default 5) |
| `rogue dwell <ms>` | Passive listen time per channel, 100–1500 (default 300) |
| `rogue rssi <dBm>` | Ignore new APs weaker than this (default -90; 0 = off) |
| `rogue confirm <n>` | Scans an AP must be heard in before it is reported (default 2) |
| `rogue maxalerts <n>` | Channel messages per scan before summarising (default 5) |
| `rogue gap <secs>` | Spacing between channel messages (default 5) |
| `rogue heartbeat <hours>` | Periodic "OK" message (default 24; 0 = off) |
| `rogue dm on\|off` | Also send DM alerts to admins (ACL alert permissions; default on) |

## BLE beacon tracker

The same firmware also listens for Bluetooth LE advertisements continuously. It shares the
2.4 GHz radio with the Wi-Fi scans using ESP32 coexistence. Any advertiser whose name starts
with one of the configured prefixes (default `RF`) is tracked automatically and reported:

```
Rogue AP Monitor: BEACON FIRST HEARD RF1234 C0:FF:EE:11:22:33 -71dBm (12 tracked)
Rogue AP Monitor: BEACON MISSING RF1234 C0:FF:EE:11:22:33 not heard for 1h00m (last -74dBm)
Rogue AP Monitor: BEACON BACK RF1234 C0:FF:EE:11:22:33 -70dBm, was missing 3h12m
```

- **FIRST HEARD** goes to the channel only. **MISSING** and **BACK** also go out as DM alerts
  when `rogue dm on` is set.
- Tracked beacons are saved to flash. After a reboot each beacon's missing timer starts again,
  and beacons that were already missing stay missing (no repeat alert).
- To retire a tag, use `beacon forget <name|mac>`. Otherwise it is reported as missing once,
  and nothing more is sent unless it comes back.
- Pick a timeout well above the beacon's advertising interval. A tag that advertises every
  ~20 minutes has three chances to be heard within the default 60 minute timeout.

| Command | Description |
|---|---|
| `beacon` / `beacon status` | Scan state, prefix, present/missing counts, adverts received |
| `beacon help` | Command summary |
| `beacon list [page]` | All tracked beacons (name, MAC, RSSI, last heard) |
| `beacon missing [page]` | Only missing beacons |
| `beacon show <name\|mac>` | Details for one beacon |
| `beacon forget <name\|mac>` | Stop tracking a beacon |
| `beacon forget missing` | Stop tracking all missing beacons |
| `beacon prefix` | Show the name prefix(es) |
| `beacon prefix RF` / `beacon prefix RF,TG` | Set one or more name prefixes (case-sensitive, comma-separated, max 31 chars). Only affects new beacons; beacons already tracked stay tracked |
| `beacon timeout <mins>` | Missing after this long without being heard (default 60) |
| `beacon rssi <dBm>` | Ignore new beacons weaker than this (default 0 = off) |
| `beacon scan active\|passive` | Active (default) asks for scan responses, which is needed if the tag only puts its name in the scan response |
| `beacon duty <5..100>` | BLE listen duty cycle (default 90). Lower uses less power but can miss short adverts |
| `beacon key mac\|name` | Track by MAC (default) or by name. Use `name` if the tags rotate their address and every name is unique |
| `beacon on` / `beacon off` | Turn BLE tracking on or off |
| `beacon reset confirm` | Forget all tracked beacons |

## Toy reactions (`pika`)

Two GPIOs can drive a toy's button contacts. The original is a Pikachu with a "pika" paw and a "cheeks light up" paw.

- **Reactions:** the toy reacts to monitor events. While any beacon is missing, the cheeks blink every `nag` minutes until someone acknowledges it.
- **Paw squeezes:** squeezing the **pika** paw posts a status line on the channel. Squeezing the **cheeks** paw acknowledges missing beacons.
- **Channel commands:** anyone on the report channel can send `!pika`, `!cheeks` or `!status`.

**Wiring, `direct` mode** (Heltec V2 default). This works for toys whose button pads sit at 3.3 V or less and are pulled to GND when pressed:

```
Heltec GPIO32 --[1k]-- toy LEFT pad  (pika)
Heltec GPIO33 --[1k]-- toy RIGHT pad (cheeks)
Heltec GND    -------- toy GND (battery -)
```

The firmware only ever drives these pins LOW, never HIGH. Between presses the pins are inputs, which is how they sense real paw squeezes.

**Wiring, `opto` mode** (any toy, fully isolated). Put a PC817 optocoupler's transistor across each contact: collector to the pad, emitter to the toy's GND. Drive its LED from GPIO through 330 Ω to GND. Then set separate paw input pins with `pika paws <say> <cheeks>`; each is a switch to Heltec GND.

| Command | Description |
|---|---|
| `pika` | Status |
| `pika say` / `pika cheeks` | Press now (test). Only one contact is ever pressed at a time |
| `pika react` | Show event → action map |
| `pika react <newap\|changed\|first\|missing\|back> <none\|say\|cheeks>` | Set a reaction. Defaults: Wi-Fi events → cheeks, beacon events → say |
| `pika nag <mins>` | Repeat the `missing` reaction while beacons are missing (default 30; 0 = off) |
| `pika ack` | Silence the nag until the next beacon goes missing (same as squeezing the cheeks paw) |
| `pika mute <mins>` / `pika unmute` | Quiet time |
| `pika on` / `pika off` | Turn all toy actions on or off |
| `pika mode direct\|opto` | Drive mode |
| `pika pins <say> <cheeks>` | Output GPIOs (-1 = unused). Board pins are refused |
| `pika paws <say> <cheeks>` | Paw input GPIOs (opto mode only) |
| `pika press <ms>` | How long a press is held (default 250) |
| `pika gap <ms>` | Minimum time between presses (default 3000) |
| `pika level high\|low` | Opto mode only: GPIO level that turns the opto on |
| `pika passthru on\|off` | Opto mode only: paw squeezes also press the toy |
| `pika chan on\|off` | Turn the channel `!` commands on or off |

## Stealth and power (Heltec V2 in a toy)

| Command | Description |
|---|---|
| `stealth` | Show the stealth setting |
| `stealth on` | OLED off and LoRa TX LED disabled. Survives reboots, and the device boots dark |
| `stealth off` | Display and TX LED back on, for troubleshooting |

Stealth can't hide the V2's orange charge LED (a hardware LED). With power on the 5 V pin and no LiPo attached, it may glow or flicker; cover it or desolder it.

The V2 build runs the CPU at 80 MHz. Rough current budget (not measured):

| Part | Approx. current |
|---|---|
| ESP32 at 80 MHz + LoRa RX | ~35 mA |
| BLE scanning at 90% duty | +60–90 mA (the biggest consumer) |
| Wi-Fi scan, ~4 s every 5 min | ~+1.5 mA average |

`beacon duty` is the main power setting, but only lower it if the tags advertise often (check `beacon show <tag>`: a high `heard` count means frequent adverts). Measure with a USB power meter before trusting any battery life estimate.

## Limitations

- **2.4 GHz only.** The ESP32-S3 can't receive 5 GHz, so an AP that runs only on 5 GHz is not
  seen. Printer Wi-Fi Direct and setup networks (`DIRECT-xx-...`) are almost always on 2.4 GHz.
- Some devices rotate their BSSID when they reboot or re-pair. An SSID or OUI allow pattern
  stops those from alerting again and again.
- The table holds 512 BSSIDs (`ROGUE_MAX_APS`). You get a warning on the channel if it fills up.
- `start ota` pauses scanning while the OTA access point is up.
- BLE tracking keeps running during Wi-Fi scans, but gets less airtime for those few seconds.
  A tag that only advertises for a moment could occasionally be missed in that window.
  The missing timeout covers several advertising intervals, which absorbs this.
- With `beacon key mac`, tags that use rotating random addresses show up as new beacons each time.
  Use `beacon key name` for those.
- Channel messages are flooded and unacknowledged. DM alerts are acknowledged and retried, but
  only reach admins who have logged in at least once.
