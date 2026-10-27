#include "SensorMesh.h"
#include "RogueApScanner.h"
#include "BeaconTracker.h"
#include "PikaToy.h"

#define NUM_ROGUE_ALERTS  3   // SensorMesh queues max 4 alerts, leave one for battery alerts
#define CHANNEL_CMD_MIN_MILLIS   5000    // rate limit for !pika / !cheeks / !status
#define PAW_STATUS_MIN_MILLIS   30000    // rate limit for paw-triggered status posts

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

// stealth: display and TX LED off, so the device looks like nothing (eg. hidden in a toy)
#define STEALTH_FILE  "/stealth"
static bool stealth = false;
static FILESYSTEM* stealth_fs = NULL;
#ifdef DISPLAY_CLASS
static bool display_started = false, ui_started = false;
#endif
void applyStealth();

static void loadStealth(FILESYSTEM* fs) {
  stealth_fs = fs;
  File f = fs->open(STEALTH_FILE, "r");
  if (f) {
    stealth = f.read() == 1;
    f.close();
  }
}

static void setStealth(bool on) {
  stealth = on;
  File f = stealth_fs->open(STEALTH_FILE, "w", true);
  if (f) {
    f.write((uint8_t)(on ? 1 : 0));
    f.close();
  }
  applyStealth();
}

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables),
       battery_data(12*24, 5*60),    // 24 hours worth of battery data, every 5 minutes
       last_channel_cmd(0), last_paw_status(0)
  {
  }

  // one-line summary, posted on the report channel
  void postStatus(const char* prefix) {
    int present, missing;
    beacons.getCounts(present, missing);
    char msg[ROGUE_MAX_TEXT + 1];
    snprintf(msg, sizeof(msg), "%s %s, %d known APs, %d unacked; beacons %d present, %d missing",
             prefix, rogue.isLearning() ? "learning" : "monitoring", rogue.numKnown(), rogue.numReported(), present, missing);
    rogue.queueText(msg);
  }

  // paws and toy outputs
  void pikaLoop() {
    int present, missing;
    beacons.getCounts(present, missing);
    pika.loop(missing);

    int paw = pika.popPawEvent();
    if (paw == PIKA_PAW_EVENT_SAY) {   // squeeze 'say' paw: post status
      if (last_paw_status == 0 || millis() - last_paw_status > PAW_STATUS_MIN_MILLIS) {
        last_paw_status = millis();
        postStatus("Pika!");
      }
    } else if (paw == PIKA_PAW_EVENT_CHEEKS && missing > 0) {   // squeeze 'cheeks' paw: acknowledge
      char msg[64];
      snprintf(msg, sizeof(msg), "Pika: acknowledged %d missing beacon(s)", missing);
      rogue.queueText(msg);
    }
  }

  // forward rogue AP / beacon reports to the configured channel, and as DM alerts to admins
  void rogueLoop() {
    char text[ROGUE_MAX_TEXT + 1];
    if (rogue.popChannelText(text) && rogue.hasChannel()) {
      sendGroupText(rogue.getChannel(), text);
    }
    if (rogue.hasAlertText()) {
      for (int i = 0; i < NUM_ROGUE_ALERTS; i++) {   // need an alert slot that has finished sending
        Trigger& t = rogue_alerts[i];
        if (isAlertQueued(t)) continue;
        rogue.popAlertText(text);
        alertIf(false, t, HIGH_PRI_ALERT, "");   // reset slot
        alertIf(true, t, HIGH_PRI_ALERT, text);
        break;
      }
    }
  }

protected:
  /* ========================== custom logic here ========================== */
  unsigned long last_channel_cmd, last_paw_status;
  Trigger low_batt, critical_batt;
  Trigger rogue_alerts[NUM_ROGUE_ALERTS];
  TimeSeriesData  battery_data;

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "stealth") == 0) {
      sprintf(reply, "stealth: %s", stealth ? "on" : "off");
      return true;
    }
    if (strcmp(command, "stealth on") == 0 || strcmp(command, "stealth off") == 0) {
      setStealth(strcmp(command, "stealth on") == 0);
      strcpy(reply, stealth ? "OK - display and LED off" : "OK - display and LED on");
      return true;
    }
    return rogue.handleCommand(command, reply) || beacons.handleCommand(command, reply) || pika.handleCommand(command, reply);
  }

  // listen on the report channel, for !pika / !cheeks / !status
  int searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel channels[], int max_matches) override {
    if (max_matches < 1 || !rogue.hasChannel() || !pika.channelCmdsEnabled()) return 0;
    if (memcmp(hash, rogue.getChannel().hash, PATH_HASH_SIZE) != 0) return 0;
    channels[0] = rogue.getChannel();
    return 1;
  }

  void onGroupDataRecv(mesh::Packet* packet, uint8_t type, const mesh::GroupChannel& channel, uint8_t* data, size_t len) override {
    if (type != PAYLOAD_TYPE_GRP_TXT || len <= 5) return;

    char text[MAX_PACKET_PAYLOAD + 1];   // data: timestamp(4), flags(1), "<sender>: <msg>"
    size_t n = len - 5;
    if (n > MAX_PACKET_PAYLOAD) n = MAX_PACKET_PAYLOAD;
    memcpy(text, &data[5], n);
    text[n] = 0;

    char* msg = strstr(text, ": ");
    if (msg == NULL) return;
    msg += 2;
    while (*msg == ' ') msg++;
    int mlen = strlen(msg);
    while (mlen > 0 && msg[mlen - 1] == ' ') msg[--mlen] = 0;
    if (msg[0] != '!') return;

    if (last_channel_cmd && millis() - last_channel_cmd < CHANNEL_CMD_MIN_MILLIS) return;   // rate limit

    if (strcasecmp(msg, "!pika") == 0) {
      pika.doAction(PIKA_ACT_SAY);
    } else if (strcasecmp(msg, "!cheeks") == 0) {
      pika.doAction(PIKA_ACT_CHEEKS);
    } else if (strcasecmp(msg, "!status") == 0) {
      postStatus("Pika!");
    } else {
      return;
    }
    last_channel_cmd = millis();
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

void applyStealth() {
#ifdef P_LORA_TX_LED
  digitalWrite(P_LORA_TX_LED, LOW);
  pinMode(P_LORA_TX_LED, stealth ? INPUT : OUTPUT);   // as an input, the TX blinks can't light it
#endif
#ifdef DISPLAY_CLASS
  if (stealth) {
    if (display_started) display.turnOff();
  } else {
    if (!display_started) display_started = display.begin();
    if (display_started) {
      if (!ui_started) {
        ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
        ui_started = true;
      } else {
        display.turnOn();
      }
    }
  }
#endif
}

static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

  SPIFFS.begin(true);
  FILESYSTEM* fs = &SPIFFS;
  loadStealth(fs);   // before the display, so stealth boots dark
#ifdef P_LORA_TX_LED
  if (stealth) pinMode(P_LORA_TX_LED, INPUT);
#endif

#ifdef DISPLAY_CLASS
  if (!stealth && (display_started = display.begin())) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Rogue AP sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);
  rogue.begin(fs, &rtc_clock);
  beacons.begin(fs, &rtc_clock, &rogue);
  pika.begin(fs);

  applyStealth();   // starts the UI when not in stealth

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  rogue.loop();
  beacons.loop();
  the_mesh.rogueLoop();
  the_mesh.pikaLoop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  if (!stealth && ui_started) ui_task.loop();
#endif
  rtc_clock.tick();
#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif
}
