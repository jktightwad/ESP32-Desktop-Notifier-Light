// =====================================================================
// On Air Light (ESP32-C3)
// Version: 1.8.7-c3
//  - Split UI:  "/" (simple)  +  "/advanced" (heavy cards)
//  - Both pages stream in chunks (low RAM, fast clicks)
//  - HW-139 touch on GPIO10: tap = on/off; 1/2/3 taps + hold = cycle
//    fun modes / presence / fast comet colours (1s per step), release keeps
//  - TDB05LFPN buzzer on GPIO7: touch feedback + AIO {"beep":n}
//  - Keeps: AIO instant re-subscribe, LED Gap, Last AIO msg, OTA, etc.
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#ifndef WM_NO_GLOBAL_DEBUG
#define WM_NO_GLOBAL_DEBUG
#endif
#include <WiFiManager.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ElegantOTA.h>
#include <math.h>
#include <algorithm>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <secrets.h>
#include "wifi_store.h"

// ---------------------- Firmware & Defaults ---------------------------
#define FIRMWARE_VERSION   "1.8.7-c3"
#define DEFAULT_LED_PIN    3
#define DEFAULT_LED_COUNT  18
#define DEFAULT_LED_GAP    0          // virtual gap length for ring animations

// HW-139 (TTP223) touch module: SIG -> GPIO10, VCC -> 3V3, GND -> GND.
// GPIO10 avoids the C3 strapping pins (2/8/9) and the UART0 pins (20/21).
// Module default (jumpers open) = momentary, HIGH while touched.
#define TOUCH_PIN          10
#define TOUCH_ACTIVE       HIGH
#define TOUCH_DEBOUNCE_MS  30         // ignore edges shorter than this
#define TOUCH_GAP_MS       400        // max gap between taps of one gesture
#define TOUCH_HOLD_MS      600        // after 1-3 taps, holding this long starts a cycle
#define TOUCH_STEP_MS      1000       // cycle: time per mode
#define COMET_FAST_SPEED   85         // UI speed (0-100) for the 3-tap colour comet

// TDB05LFPN magnetic buzzer (5V / ~30mA) on GPIO7 via NPN transistor (not direct).
// Bench-tested: steady tone on plain 5V DC = built-in oscillator, so drive it on/off.
// Beep timing runs on an esp_timer, so it stays short even if loop() is blocked.
#define BUZZER_PIN         7
#define BEEP_TAP_MS        15         // on/off and cycle confirm


// ---------------------- Adafruit IO (AIO) -----------------------------
static const char* AIO_BROKER        = "io.adafruit.com";
static const uint16_t AIO_PORT       = 1883;
static const char* AIO_FEED_CONTROL  = "onair-dot-control";
static const char* AIO_FEED_DEVICES  = "debug-messages";   // existing shared feed (key from AIO feed page)
 
// ---------------------- Files in LittleFS -----------------------------
static const char* CFG_FILE        = "/config.txt";       // deviceName
static const char* AIO_DEVICE_FILE = "/aio_device.txt";   // AIO custom device ID
static const char* GROUPS_FILE     = "/groups.txt";       // AIO group list
static const char* FORGET_FILE     = "/forget.flag";      // one-shot: force portal next boot
static const char* OTAONLY_FILE    = "/otaonly.flag";     // recovery flag
static const char* HW_FILE         = "/hw.txt";           // "pin,count,gap"
static const char* LASTMSG_FILE    = "/lastmsg.json";     // persists last AIO message
 
// ---------------------- Globals --------------------------------------
static uint16_t gLedCount = DEFAULT_LED_COUNT;
static uint8_t  gLedPin   = DEFAULT_LED_PIN;
static uint16_t gLedGap   = DEFAULT_LED_GAP;
 
Adafruit_NeoPixel strip(DEFAULT_LED_COUNT, DEFAULT_LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);
WiFiManager wm;
 
WiFiClient espClient;
PubSubClient mqtt(espClient);
 
char deviceName[41] = "On Air Light";
String aioDeviceId  = "unset";
#include <vector>
std::vector<String> groups;
 
// Last AIO message (for UI)
static String   lastMsgTopic   = "-";
static String   lastMsgPayload = "-";
static uint32_t lastMsgMs      = 0;
 
// UI / effects
enum effect { OFF, SOLID, BLINK, PARTY, KNIGHT_RIDER, BREATHE, COMET, RAINBOW, PULSE };
effect currentEffect = OFF;      // Boot OFF
bool   powerState    = false;    // Boot OFF
 
uint32_t currentColor = 0;
float    targetHue = 0.0f, targetSaturation = 1.0f; // full V; brightness via strip
unsigned long effectDelay = 100;  // set to medium (UI 50) in setup()
unsigned long lastUpdate  = 0;
int effectPosition = 0;
int effectDirection = 1;
uint8_t globalBrightness = 255;
 
bool pulseResetRequested = false;
 
// Discovery blink (loop-serviced)
enum BlinkMode { BLINK_NONE, BLINK_ALL_PURPLE, BLINK_MIDDLE6_PURPLE };
BlinkMode blinkMode = BLINK_NONE;
bool     blinkActive = false;
uint32_t blinkNextMs = 0;
bool     blinkPhase = false;
 
bool     apRunning = false;
uint32_t apOffAtMs = 0;
 
unsigned long lastHeartbeatMs = 0;
static bool statusActive = false;
static bool discoveryBlinkActive = false;
static void beep(uint8_t count=1, uint16_t onMs=60, uint16_t offMs=80);   // buzzer, defined below
 
// ---------------------- Helpers --------------------------------------
static inline uint32_t RGB(uint8_t r,uint8_t g,uint8_t b){ return strip.Color(r,g,b); }
static inline String colorToHex(uint32_t c){ char s[8]; sprintf(s,"#%02x%02x%02x",(c>>16)&0xFF,(c>>8)&0xFF,c&0xFF); return String(s); }
static inline String toLowerTrim(const String& s){ String t=s; t.trim(); t.toLowerCase(); return t; }
static inline String cleanToken(const String& in){
  String out;
  for (char c: in){ if (c>='A'&&c<='Z') c=c-'A'+'a'; if ((c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_') out+=c; }
  out.trim(); return out;
}
static inline int brightnessPctFrom255(uint8_t gb){ return (int)((gb*100UL+127)/255UL); }
static inline uint8_t brightness255FromPct(int pct){ pct=constrain(pct,0,100); return (uint8_t)((pct*255UL+50)/100UL); }
 
static inline void setAllPixels(uint32_t col){
  strip.setBrightness(globalBrightness);
  for(int i=0;i<strip.numPixels();++i) strip.setPixelColor(i,col);
  strip.show();
}
// WiFi.setAutoConnect() was removed in Arduino-ESP32 core 3.x; no-op there.
static inline void wifiSetAutoConnect(bool on){
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
  WiFi.setAutoConnect(on);
#else
  (void)on;
#endif
}
static String macString(){
  uint64_t mac = ESP.getEfuseMac();
  uint8_t m[6] = {(uint8_t)(mac>>40),(uint8_t)(mac>>32),(uint8_t)(mac>>24),(uint8_t)(mac>>16),(uint8_t)(mac>>8),(uint8_t)mac};
  char buf[18]; sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", m[0],m[1],m[2],m[3],m[4],m[5]); return String(buf);
}
 
// HSV converters
static void rgbToHsv(uint32_t rgbc, float &h, float &s, float &v){
  float r=((rgbc>>16)&0xFF)/255.0f, g=((rgbc>>8)&0xFF)/255.0f, b=(rgbc&0xFF)/255.0f;
  float mx=max(max(r,g),b), mn=min(min(r,g),b), d=mx-mn; v=mx;
  if (d==0){ h=0; s=0; }
  else{
    s=d/mx;
    if      (mx==r) h=(g-b)/d + (g<b?6:0);
    else if (mx==g) h=(b-r)/d + 2;
    else            h=(r-g)/d + 4;
    h/=6;
  }
}
static uint32_t hsvToRgb(float h, float s, float v){
  int i=int(h*6);
  float f=h*6-i, p=v*(1-s), q=v*(1-f*s), t=v*(1-(1-f)*s);
  float rr=0,gg=0,bb=0;
  switch(i%6){
    case 0: rr=v; gg=t; bb=p; break;
    case 1: rr=q; gg=v; bb=p; break;
    case 2: rr=p; gg=v; bb=t; break;
    case 3: rr=p; gg=q; bb=v; break;
    case 4: rr=t; gg=p; bb=v; break;
    case 5: rr=v; gg=p; bb=q; break;
  }
  return RGB(uint8_t(rr*255),uint8_t(gg*255),uint8_t(bb*255));
}
static void applyColorPreserveBrightness(uint8_t r, uint8_t g, uint8_t b){
  float h,s,v_ignored;
  rgbToHsv(RGB(r,g,b), h, s, v_ignored);
  targetHue = h; targetSaturation = s;
  currentColor = hsvToRgb(targetHue, targetSaturation, 1.0f); // full-V
}
 
// ---------------------- FS -------------------------------------------
static void saveConfig(){ File f=LittleFS.open(CFG_FILE,"w"); if(!f) return; f.println(deviceName); f.close(); }
static void loadConfig(){ if(!LittleFS.exists(CFG_FILE)) return; File f=LittleFS.open(CFG_FILE,"r"); if(!f) return; String nm=f.readStringUntil('\n'); nm.trim(); if(nm.length()){ strncpy(deviceName,nm.c_str(),sizeof(deviceName)); deviceName[sizeof(deviceName)-1]='\0'; } f.close(); }
static void loadAIODevice(){ if(!LittleFS.exists(AIO_DEVICE_FILE)) return; File f=LittleFS.open(AIO_DEVICE_FILE,"r"); if(!f) return; String v=f.readStringUntil('\n'); v.trim(); if(v.length()) aioDeviceId=v; f.close(); }
static void saveAIODevice(){ File f=LittleFS.open(AIO_DEVICE_FILE,"w"); if(!f) return; f.println(aioDeviceId); f.close(); }
static void loadGroups(){ groups.clear(); if(!LittleFS.exists(GROUPS_FILE)) return; File f=LittleFS.open(GROUPS_FILE,"r"); if(!f) return; while(f.available()){ String g=f.readStringUntil('\n'); g.trim(); if(g.length()) groups.push_back(g); } f.close(); }
static void saveGroups(){ File f=LittleFS.open(GROUPS_FILE,"w"); if(!f) return; for(auto &g:groups) f.println(g); f.close(); }
 
// Hardware cfg: "pin,count,gap"
static void loadHardwareCfg(){
  gLedPin   = DEFAULT_LED_PIN;
  gLedCount = DEFAULT_LED_COUNT;
  gLedGap   = DEFAULT_LED_GAP;
  if (!LittleFS.exists(HW_FILE)) return;
  File f = LittleFS.open(HW_FILE, "r"); if (!f) return;
  String line = f.readStringUntil('\n'); f.close(); line.trim();
  if (!line.length()) return;
  int p1 = line.indexOf(','), p2 = line.indexOf(',', p1+1);
  if (p1<0 || p2<0) return;
  int pin  = line.substring(0,p1).toInt();
  int cnt  = line.substring(p1+1,p2).toInt();
  int gap  = line.substring(p2+1).toInt();
  if (pin>=0 && pin<=21) gLedPin = (uint8_t)pin;
  if (cnt>=1 && cnt<=600) gLedCount = (uint16_t)cnt;
  if (gap>=0 && gap<=gLedCount*2) gLedGap = (uint16_t)gap;
}
static void saveHardwareCfg(){
  File f = LittleFS.open(HW_FILE, "w"); if (!f) return;
  f.printf("%u,%u,%u\n", (unsigned)gLedPin, (unsigned)gLedCount, (unsigned)gLedGap);
  f.close();
}
 
// Persist last message
static void saveLastMsg(){
  File f = LittleFS.open(LASTMSG_FILE, "w"); if (!f) return;
  JsonDocument d;
  d["topic"]   = lastMsgTopic;
  d["payload"] = lastMsgPayload;
  d["ms"]      = lastMsgMs;
  serializeJson(d, f);
  f.close();
}
static void loadLastMsg(){
  lastMsgTopic="-"; lastMsgPayload="-"; lastMsgMs=0;
  if (!LittleFS.exists(LASTMSG_FILE)) return;
  File f = LittleFS.open(LASTMSG_FILE, "r"); if (!f) return;
  JsonDocument d; DeserializationError e = deserializeJson(d, f); f.close();
  if (!e){
    if (d["topic"].is<const char*>())   lastMsgTopic = d["topic"].as<const char*>();
    if (d["payload"].is<const char*>()) lastMsgPayload = d["payload"].as<const char*>();
    if (d["ms"].is<uint32_t>())         lastMsgMs = d["ms"].as<uint32_t>();
  }
}
 
// Reinit the strip with current hw cfg
static void initStrip(){
  strip.updateLength(gLedCount);
  strip.setPin(gLedPin);
  strip.begin();
  strip.show(); // clear
}
 
// ---------------------- Timing helpers --------------------------------
static inline unsigned long delayFor(effect e){
  float k=1.0f;
  switch (e){
    case RAINBOW: k=1.0f; break;  case BREATHE: k=2.0f; break;  case PARTY: k=2.5f; break;
    case COMET: k=2.5f; break;    case KNIGHT_RIDER: k=3.0f; break; case BLINK: k=3.0f; break;
    case PULSE: k=2.0f; break;    case SOLID: default: k=1.0f; break;
  }
  unsigned long d=(unsigned long)(effectDelay*k); if(d<2UL)d=2UL; if(d>2000UL)d=2000UL; return d;
}
static inline unsigned long safeInterval(unsigned long v, unsigned long minv=2UL){ return (v<minv)?minv:v; }
 
// ---------------------- Names -----------------------------------------
static String getEffectName(){
  switch (currentEffect){
    case SOLID: return "Solid"; case BLINK: return "Blink"; case PARTY: return "Party";
    case KNIGHT_RIDER: return "Knight Rider"; case BREATHE: return "Breathe";
    case COMET: return "Comet"; case RAINBOW: return "Rainbow"; case PULSE: return "Pulse";
    case OFF: default: return "Off";
  }
}
 
// ---------------------- Progress bar ----------------------------------
static void drawProgressBar(uint32_t color, float frac){
  uint16_t n = strip.numPixels(); if (n==0) return;
  if (frac<0) frac=0; if (frac>1) frac=1;
  int filled = (int)floor(frac * n);
  for (uint16_t i=0;i<n;++i) strip.setPixelColor(i, 0);
  for (int i=0;i<filled && i<(int)n; ++i) strip.setPixelColor(i, color);
  if (filled < (int)n){
    uint8_t r=(color>>16)&0xFF, g=(color>>8)&0xFF, b=color&0xFF;
    strip.setPixelColor(filled, RGB(r/4,g/4,b/4));
  }
  strip.setBrightness(globalBrightness);
  strip.show();
}
static void drawProgressBarAndDelay(uint32_t color, uint32_t s, uint32_t ms){
  float frac = float(millis()-s) / float(ms);
  drawProgressBar(color, frac);
  delay(120);
  yield();
}
 
// ---------------------- Virtual ring helpers (gap support) ------------
static inline int virtRingLen(){ return (int)gLedCount + (int)gLedGap; }
static inline int virtToPhysical(int vpos){
  int V = virtRingLen();
  if (V<=0) return -1;
  vpos %= V; if (vpos<0) vpos += V;
  if (vpos >= (int)gLedCount) return -1;   // inside the gap
  return vpos;                              // direct mapping (gap at the end)
}
 
// ---------------------- Effects --------------------------------------
static void handleBlink(){
  unsigned long d = delayFor(BLINK);
  if (millis()-lastUpdate > (d*4+100)){
    lastUpdate=millis();
    strip.setBrightness(globalBrightness);
    if (strip.getPixelColor(0)==0) setAllPixels(currentColor); else setAllPixels(0);
  }
}
static void handleParty(){
  if (millis()-lastUpdate > delayFor(PARTY)){
    lastUpdate=millis();
    strip.setBrightness(globalBrightness);
    for (int i=0;i<strip.numPixels();++i) strip.setPixelColor(i, RGB(random(25),random(25),random(25)));
    for (int i=0;i<3;++i){ float rnd=fmod(targetHue+(random(-60,61)/360.0f)+1.0f,1.0f); strip.setPixelColor(random(strip.numPixels()), hsvToRgb(rnd,1.0f,1.0f)); }
    strip.show();
  }
}
static void handleKnightRider(){
  if (millis()-lastUpdate > delayFor(KNIGHT_RIDER)){
    lastUpdate=millis();
    strip.setBrightness(globalBrightness);
    for (int i=0;i<strip.numPixels();++i){
      uint32_t c=strip.getPixelColor(i);
      uint8_t r=((c>>16)&0xFF)*0.7, g=((c>>8)&0xFF)*0.7, b=(c&0xFF)*0.7;
      strip.setPixelColor(i, RGB(r,g,b));
    }
    uint8_t r=(currentColor>>16)&0xFF, g=(currentColor>>8)&0xFF, b=currentColor&0xFF;
    if (effectPosition>1) { strip.setPixelColor(effectPosition-2, RGB(r/8,g/8,b/8)); }
    if (effectPosition>0) { strip.setPixelColor(effectPosition-1, RGB(r/2,g/2,b/2)); }
    strip.setPixelColor(effectPosition, RGB(r,g,b));
    if (effectPosition<strip.numPixels()-1) { strip.setPixelColor(effectPosition+1, RGB(r/2,g/2,b/2)); }
    if (effectPosition<strip.numPixels()-2) { strip.setPixelColor(effectPosition+2, RGB(r/8,g/8,b/8)); }
    strip.show();
    effectPosition+=effectDirection;
    if (effectPosition<0 || effectPosition>=strip.numPixels()){ effectDirection*=-1; effectPosition+=effectDirection; }
  }
}
static void handleBreathe(){
  static float phase=0.0f;
  unsigned long d = delayFor(BREATHE);
  unsigned long interval = safeInterval(d/4, 3UL);
  if (millis()-lastUpdate > interval){
    lastUpdate=millis();
    strip.setBrightness(globalBrightness);
    phase += 0.03f;
    float v = 0.2f + 0.8f*(0.5f*(1.0f+sin(phase)));
    setAllPixels(hsvToRgb(targetHue,targetSaturation,v));
  }
}
static void handleComet(){
  if (millis()-lastUpdate > delayFor(COMET)){
    lastUpdate=millis();
    strip.setBrightness(globalBrightness);
    for (int i=0;i<strip.numPixels();++i){
      uint32_t c=strip.getPixelColor(i);
      uint8_t r=((c>>16)&0xFF)*0.8, g=((c>>8)&0xFF)*0.8, b=(c&0xFF)*0.8;
      strip.setPixelColor(i, RGB(r,g,b));
    }
    static int vpos = 0;
    int phys = virtToPhysical(vpos);
    uint8_t r=(currentColor>>16)&0xFF, g=(currentColor>>8)&0xFF, b=currentColor&0xFF;
    if (phys >= 0) strip.setPixelColor(phys, RGB(r,g,b));
    strip.show();
    vpos = (vpos + 1) % max(1, virtRingLen());
  }
}
// RAINBOW: original "feel" with gap-aware mapping
static void handleRainbow(){
  static uint16_t base = 0;
  unsigned long interval = max<unsigned long>(2, effectDelay / 16);
  if (millis() - lastUpdate > interval){
    lastUpdate = millis();
    strip.setBrightness(globalBrightness);
    const unsigned long ed = min<unsigned long>(effectDelay, 200UL);
    const uint8_t step = 24 + (uint8_t)((220 - (uint8_t)ed) * 0.6f);
    base = (base + step) & 0xFFFF;
 
    const int V = max(1, virtRingLen());
    for (int i = 0; i < (int)gLedCount; ++i){
      const int v = (int)(( (long)i * V ) / (long)gLedCount);
      const uint16_t vphase = (uint32_t)v * 65535UL / (uint32_t)V;
      const float h = ((uint16_t)(base + vphase)) / 65535.0f;
      strip.setPixelColor(i, hsvToRgb(h, 1.0f, 1.0f));
    }
    strip.show();
  }
}
static void handlePulse() {
  static int pPos = 0, pDir = 1;
  if (pulseResetRequested) { pPos = 0; pDir = 1; pulseResetRequested = false; }
  if (millis() - lastUpdate > delayFor(PULSE)) {
    lastUpdate = millis();
    strip.setBrightness(globalBrightness);
    const int n = strip.numPixels(); if (n <= 0) return;
    for (int i=0;i<n;i++){
      uint32_t c=strip.getPixelColor(i);
      uint8_t r=((c>>16)&0xFF)*7/10, g=((c>>8)&0xFF)*7/10, b=(c&0xFF)*7/10;
      strip.setPixelColor(i, strip.Color(r,g,b));
    }
    const int left=pPos, right=(n-1)-pPos;
    uint8_t r=(currentColor>>16)&0xFF, g=(currentColor>>8)&0xFF, b=currentColor&0xFF;
    if (left  >=0 && left  < n) strip.setPixelColor(left,  strip.Color(r,g,b));
    if (right >=0 && right < n) strip.setPixelColor(right, strip.Color(r,g,b));
    if (left-1  >= 0) strip.setPixelColor(left-1,  strip.Color(r/2,g/2,b/2));
    if (right+1 <  n) strip.setPixelColor(right+1, strip.Color(r/2,g/2,b/2));
    bool oddCenter = (left==right), evenNeck = ((right-left)==1);
    if (!oddCenter && !evenNeck){
      if (left+1  <  n) strip.setPixelColor(left+1,  strip.Color(r/3,g/3,b/3));
      if (right-1 >= 0) strip.setPixelColor(right-1, strip.Color(r/3,g/3,b/3));
    }
    strip.show();
    if (pDir > 0) { if (oddCenter || evenNeck) pDir = -1; else pPos++; }
    else { if (pPos > 0) pPos--; else { pPos=0; pDir=1; } }
  }
}
 
// ---------------------- Blink service --------------------------------
static void serviceBlink(){
  if (!blinkActive) return;
  if (millis() < blinkNextMs) return;
  blinkNextMs = millis() + 400;
  blinkPhase = !blinkPhase;
  setAllPixels(0);
  if (blinkPhase){
    if (blinkMode==BLINK_ALL_PURPLE){ setAllPixels(RGB(128,0,255)); }
    else if (blinkMode==BLINK_MIDDLE6_PURPLE){
      int n=strip.numPixels(), mid=n/2;
      strip.setBrightness(globalBrightness);
      for (int i=mid-3;i<=mid+2 && i<n;++i) if (i>=0) strip.setPixelColor(i, RGB(128,0,255));
      strip.show();
    }
  }
}
static void startBlink(BlinkMode m){ blinkMode=m; blinkActive=true; blinkNextMs=millis(); blinkPhase=false; }
static void stopBlink(){ blinkActive=false; setAllPixels(0); }
static void startDiscoveryBlink() { discoveryBlinkActive = true; startBlink(BLINK_MIDDLE6_PURPLE); }
static void stopDiscoveryBlink(bool keepAp = true) {
  stopBlink(); discoveryBlinkActive = false;
  if (!keepAp && apRunning) { WiFi.softAPdisconnect(true); apRunning = false; }
}
static void startEphemeralAPWithStaIP(uint32_t durationMs=60000){
  WiFi.softAPdisconnect(true); apRunning = false;
  String suffix="chip"; suffix+=String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF),HEX);
  if (WiFi.status()==WL_CONNECTED){ suffix = WiFi.localIP().toString(); suffix.replace(".","-"); }
  String apName = String("OnAirLight-") + suffix;
  WiFi.softAP(apName.c_str()); apRunning = true; apOffAtMs = millis() + durationMs;
  startDiscoveryBlink();
  Serial.printf("[AP] Discovery SSID \"%s\" up for %lus\n", apName.c_str(), durationMs/1000UL);
}
 
// ---------------------- Wi-Fi connect paths ---------------------------
static void showPortalStaticPurple(bool on){
  setAllPixels(0);
  if (on){
    int n=strip.numPixels(), mid=n/2;
    strip.setBrightness(globalBrightness);
    for (int i=mid-3;i<=mid+2 && i<n;++i) if (i>=0) strip.setPixelColor(i, RGB(128,0,255));
    strip.show();
  }
}
// Known networks: the ones this device has joined (/wifi.txt, most recent first),
// then any listed in secrets.h (WIFI_NETWORKS and/or PRIMARY_WIFI_SSID).
#ifdef WIFI_NETWORKS
struct BuiltinNet { const char* ssid; const char* pass; };
static const BuiltinNet BUILTIN_NETWORKS[] = WIFI_NETWORKS;
#endif
static WifiStore wifiStore;

static void addKnown(std::vector<WifiCred>& v, const String& ssid, const String& pass){
  if (!ssid.length()) return;
  for (const WifiCred& k : v) if (k.ssid == ssid) return;
  v.push_back({ssid, pass});
}
static std::vector<WifiCred> knownNetworks(){
  std::vector<WifiCred> v = wifiStore.networks();
#ifdef WIFI_NETWORKS
  for (const BuiltinNet& n : BUILTIN_NETWORKS) addKnown(v, n.ssid, n.pass);
#endif
#ifdef PRIMARY_WIFI_SSID
  addKnown(v, PRIMARY_WIFI_SSID, PRIMARY_WIFI_PASS);
#endif
  return v;
}
// The network the ESP32 saved itself (earlier firmware or the portal), copied into the list.
static void importEspSavedNetwork(){
  wifi_config_t conf;
  if (esp_wifi_get_config(WIFI_IF_STA, &conf) != ESP_OK) return;
  String ssid = String((const char*)conf.sta.ssid);
  if (!ssid.length()) return;
  for (const WifiCred& n : wifiStore.networks()) if (n.ssid == ssid) return;
  wifiStore.remember(ssid, String((const char*)conf.sta.password));
}
static void rememberCurrentNetwork(){
  if (WiFi.status()==WL_CONNECTED) wifiStore.remember(WiFi.SSID(), WiFi.psk());
}
// Background scan so the blue bar keeps moving. Returns networks found, or <0 on failure.
static int scanWithBar(bool showBar, uint32_t ms=10000){
  WiFi.scanDelete();
  WiFi.scanNetworks(true);
  uint32_t s=millis(); int n;
  while ((n = WiFi.scanComplete()) == WIFI_SCAN_RUNNING && millis()-s<ms){
    if (showBar) drawProgressBarAndDelay(RGB(0,0,255), s, ms); else delay(50);
  }
  return n;
}
// Tries known networks in range, strongest first, then the ones the scan didn't see
// (hidden networks never show up in a scan). Blue bar while scanning, orange while
// joining each one. showBar=false leaves the current effect alone (used from loop()).
static bool tryKnownNetworks(bool showBar=true, uint32_t msEach=15000, uint32_t msUnseen=8000){
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Already connected: %s | IP:%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    rememberCurrentNetwork(); return true;
  }
  std::vector<WifiCred> known = knownNetworks();
  if (known.empty()){ Serial.println("[WiFi] No known networks."); return false; }
  if (showBar) statusActive = true;
  Serial.printf("[WiFi] Scanning for %u known network(s)...\n", (unsigned)known.size());
  WiFi.disconnect();   // stop any auto-connect attempt so the scan can run
  int found = scanWithBar(showBar);
  if (found < 0) Serial.println("[WiFi] Scan failed.");

  const int UNSEEN = -1000;
  struct Candidate { size_t idx; int rssi; };
  std::vector<Candidate> cands;
  for (size_t k=0; k<known.size(); ++k){
    int best = UNSEEN;
    for (int i=0; i<found; ++i) if (WiFi.SSID(i) == known[k].ssid && WiFi.RSSI(i) > best) best = WiFi.RSSI(i);
    cands.push_back({k, best});
  }
  WiFi.scanDelete();
  // Seen networks strongest first; unseen ones keep list order (most recent first) at the end.
  std::stable_sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b){ return a.rssi > b.rssi; });

  for (const Candidate& c : cands){
    const WifiCred& n = known[c.idx];
    const bool seen = c.rssi != UNSEEN;
    const uint32_t ms = seen ? msEach : msUnseen;
    if (seen) Serial.printf("[WiFi] Trying \"%s\" (%d dBm)...\n", n.ssid.c_str(), c.rssi);
    else      Serial.printf("[WiFi] Trying \"%s\" (not seen in scan, may be hidden)...\n", n.ssid.c_str());
    WiFi.begin(n.ssid.c_str(), n.pass.c_str());
    uint32_t s=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-s<ms){
      if (showBar) drawProgressBarAndDelay(RGB(255,128,0), s, ms); else delay(120);
    }
    if (WiFi.status()==WL_CONNECTED){
      Serial.printf("[WiFi] Connected: %s | IP:%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      rememberCurrentNetwork();
      if (showBar){ setAllPixels(0); statusActive = false; }
      return true;
    }
    Serial.printf("[WiFi] \"%s\" failed.\n", n.ssid.c_str());
    WiFi.disconnect();
  }
  if (showBar){ setAllPixels(0); statusActive = false; }
  return false;
}
// From loop(): saves networks joined later, and if Wi-Fi stays down, looks for any
// other known network once a minute (e.g. the device was carried to another site).
static void serviceWifi(){
  static bool wasConnected = true;
  static uint32_t lostAtMs = 0, lastRetryMs = 0;
  if (!(WiFi.getMode() & WIFI_STA)) return;   // OTA recovery runs AP-only
  bool connected = WiFi.status()==WL_CONNECTED;
  if (connected != wasConnected){
    wasConnected = connected;
    if (connected) rememberCurrentNetwork();
    else { Serial.println("[WiFi] Disconnected."); lostAtMs = millis(); }
  }
  if (!connected && millis()-lostAtMs > 60000 && millis()-lastRetryMs > 60000){
    lastRetryMs = millis();
    Serial.println("[WiFi] Still down - scanning for known networks");
    tryKnownNetworks(false);
  }
}
static void afterConnectAnnounce(){ startEphemeralAPWithStaIP(60000); }
static void connectWithFallbacks(){
  importEspSavedNetwork();
  if (tryKnownNetworks()){ afterConnectAnnounce(); return; }
  WiFi.softAPdisconnect(true); apRunning = false;
  Serial.println("[WiFi] Launching Config Portal (OPEN, WiFiManager)...");
  wm.setConfigPortalTimeout(300);
  wm.setAPCallback([](WiFiManager *){ showPortalStaticPurple(true); });
  bool ok = wm.startConfigPortal("OnAirLight-Setup");
  showPortalStaticPurple(false);
  if (ok){
    setAllPixels(RGB(0,255,0)); delay(120); setAllPixels(0);
    Serial.printf("[WiFi] Connected via portal. SSID=%s IP=%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    rememberCurrentNetwork();
    afterConnectAnnounce();
  } else {
    Serial.println("[WiFi] Portal timeout.");
  }
}
static void startConfigPortalOnly(){
  Serial.println("[WiFi] Forget flag present — starting Config Portal (OPEN).");
  WiFi.mode(WIFI_STA); WiFi.persistent(true);
  wifiSetAutoConnect(false); WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, true); WiFi.softAPdisconnect(true); apRunning = false; delay(100);
  wm.setConfigPortalTimeout(300);
  wm.setAPCallback([](WiFiManager *){ showPortalStaticPurple(true); });
  bool ok = wm.startConfigPortal("OnAirLight-Setup");
  showPortalStaticPurple(false);
  if (ok){
    setAllPixels(RGB(0,255,0)); delay(120); setAllPixels(0);
    Serial.printf("[WiFi] Connected via portal. SSID=%s IP=%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    rememberCurrentNetwork();
    afterConnectAnnounce();
  } else {
    Serial.println("[WiFi] Portal timeout.");
  }
}
 
// ---------------------- MQTT (AIO) -----------------------------------
static String aioTopic(const String& feedKey){ String t(AIO_USERNAME); t+="/feeds/"; t+=feedKey; return t; }
static String aioErrorsTopic()   { return String(AIO_USERNAME) + "/errors";   }
static String aioThrottleTopic() { return String(AIO_USERNAME) + "/throttle"; }
 
// Unique per device. AIO disconnects the OLDER session whenever a second
// client connects with the same client ID, so this must never collide.
// (The old ID used only the low 24 bits of getEfuseMac(), which is the
//  Espressif vendor prefix shared by every board.)
static String mqttClientId(){
  char buf[24];
  snprintf(buf, sizeof(buf), "onair-%012llX", (unsigned long long)ESP.getEfuseMac());
  return String(buf);
}
 
// Subscribe ONLY to topics that are valid on Adafruit IO.
//  - core control feed: <user>/feeds/onair-dot-control
//  - <user>/errors and <user>/throttle: AIO publishes the reason for any
//    rejection/ban here, so we print it to the serial console.
// Device and group targeting is done inside the payload by isTargeted()
// (deviceId / deviceIds / group / groups), so no per-device or per-group
// feeds are needed. Every FAILED subscribe counts toward a ban
// (10 failed SUBSCRIBEs per minute = account-wide ban for ALL devices).
static void aioSubscribeAll(){
  Serial.println(F("[AIO] Subscribing..."));
  String core = aioTopic(AIO_FEED_CONTROL);
  if (mqtt.subscribe(core.c_str())) {
    Serial.print(F("  + core feed: ")); Serial.println(core);
  } else {
    Serial.println(F("  ! core subscribe failed"));
  }
  mqtt.subscribe(aioErrorsTopic().c_str());
  mqtt.subscribe(aioThrottleTopic().c_str());
}
static bool eqId(const String& a, const String& b){ return toLowerTrim(a) == toLowerTrim(b); }
static bool isTargeted(const JsonObject& obj){
  if (obj["deviceId"].is<const char*>()){
    String did = obj["deviceId"].as<const char*>();
    if (aioDeviceId.length() && aioDeviceId!="unset" && eqId(aioDeviceId, did)) return true;
  }
  if (obj["deviceIds"].is<JsonArray>()){
    for (JsonVariant v : obj["deviceIds"].as<JsonArray>()){
      if (!v.is<const char*>()) continue;
      String d = v.as<const char*>();
      if (aioDeviceId.length() && aioDeviceId!="unset" && eqId(aioDeviceId, d)) return true;
    }
  }
  if (obj["group"].is<const char*>()){
    String g=cleanToken(obj["group"].as<const char*>());
    if (g=="all") return true;
    for (auto &x:groups) if (eqId(x,g)) return true;
  }
  if (obj["groups"].is<JsonArray>()){
    for (JsonVariant v : obj["groups"].as<JsonArray>()){
      if (!v.is<const char*>()) continue;
      String g=cleanToken(v.as<const char*>());
      if (g=="all") return true;
      for (auto &x:groups) if (eqId(x,g)) return true;
    }
  }
  return false;
}
static unsigned long speedToDelay(int s){
  int v = s; if (v > 100) v = map(v, 10, 200, 0, 100); v = constrain(v, 0, 100);
  float k = 0.035f; float d = 200.0f * expf(-k * v); if (d < 8.0f) d = 8.0f; return (unsigned long)d;
}
static void applyPresence(const String& m){
  uint8_t rr=0,gg=0,bb=0;
  if (m=="available") { rr=0;   gg=255; bb=0;   }
  else if (m=="away") { rr=0;   gg=0;   bb=255; }
  else /* busy */     { rr=255; gg=0;   bb=0;   }
  applyColorPreserveBrightness(rr,gg,bb);
  currentEffect = SOLID; powerState = true; setAllPixels(currentColor);
}
static void recordLastMsg(const String& topic, const String& payload){
  lastMsgTopic = topic; lastMsgPayload = payload; lastMsgMs = millis();
  saveLastMsg();
}
static void applyCommand(const JsonObject& obj, const String& srcTopic, const String& rawPayload){
  if (!isTargeted(obj)) return;
  recordLastMsg(srcTopic, rawPayload);
  stopDiscoveryBlink(true);
  if (obj["beep"].is<int>()) beep((uint8_t)constrain((int)obj["beep"], 1, 10), 120, 120);
 
  if (obj["brightness"].is<int>()){
    int br = constrain((int)obj["brightness"],0,100);
    globalBrightness = brightness255FromPct(br);
    if (powerState) {
      if (currentEffect==SOLID) setAllPixels(currentColor);
      else { strip.setBrightness(globalBrightness); strip.show(); if (currentEffect==PULSE) pulseResetRequested=true; }
    }
  }
  if (obj["color"].is<JsonArray>()){
    auto col = obj["color"].as<JsonArray>();
    if (col.size()>=3){
      uint8_t r=col[0], g=col[1], b=col[2];
      applyColorPreserveBrightness(r,g,b);
      if (powerState && currentEffect==SOLID) setAllPixels(currentColor);
      if (currentEffect==PULSE) pulseResetRequested = true;
    }
  }
  if (obj["speed"].is<int>()){
    effectDelay = speedToDelay((int)obj["speed"]);
    if (currentEffect==PULSE) pulseResetRequested = true;
  }
  if (obj["mode"].is<const char*>()){
    String m=obj["mode"].as<const char*>(); m.trim(); m.toLowerCase(); m.replace('-', '_');
    if (m=="available" || m=="away" || m=="busy") { applyPresence(m); return; }
    if      (m=="solid")         { currentEffect=SOLID; powerState=true; setAllPixels(currentColor); }
    else if (m=="blink")         currentEffect=BLINK;
    else if (m=="party")         currentEffect=PARTY;
    else if (m=="knight_rider" || m=="kitt") { currentEffect=KNIGHT_RIDER; effectPosition=0; effectDirection=1; }
    else if (m=="breathe")       currentEffect=BREATHE;
    else if (m=="comet")         currentEffect=COMET;
    else if (m=="rainbow")       currentEffect=RAINBOW;
    else if (m=="pulse")         { currentEffect=PULSE; pulseResetRequested = true; }
    else if (m=="off")           { currentEffect=OFF; setAllPixels(0); powerState=false; return; }
    powerState=true;
    if (currentEffect==SOLID) setAllPixels(currentColor);
  }
}
static void mqttCallback(char* topic, byte* payload, unsigned int len){
  String t = String(topic);
  String p; p.reserve(len); for (unsigned i=0;i<len;i++) p += (char)payload[i];
 
  // AIO publishes rejections, throttle warnings and bans on these topics.
  if (t.endsWith("/errors") || t.endsWith("/throttle")) {
    Serial.print(F("[AIO] !! ")); Serial.print(t); Serial.print(F(": ")); Serial.println(p);
    return;
  }
 
  Serial.print(F("[AIO] RX topic: ")); Serial.println(t);
 
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, payload, len);
  if (e) { Serial.print(F("[AIO] JSON error: ")); Serial.println(e.c_str()); return; }
 
  if (doc["value"].is<const char*>()){
    const char* inner = doc["value"].as<const char*>();
    JsonDocument d2;
    if (deserializeJson(d2, inner) == DeserializationError::Ok && d2.is<JsonObject>()){
      applyCommand(d2.as<JsonObject>(), t, String(inner));
    } else {
      Serial.println(F("[AIO] Inner JSON invalid"));
    }
    return;
  }
  if (doc.is<JsonObject>()) applyCommand(doc.as<JsonObject>(), t, p);
}
 
// ---- Connection management with backoff ----
// AIO allows 20 login attempts per minute, then bans (30s, 60s, 90s ...).
static const unsigned long MQTT_BACKOFF_MIN = 5000UL;
static const unsigned long MQTT_BACKOFF_MAX = 120000UL;
static unsigned long nextMqttAttemptMs = 0;
static unsigned long mqttBackoffMs     = MQTT_BACKOFF_MIN;
static unsigned long mqttConnectedAtMs = 0;
static bool          mqttWasConnected  = false;
static bool          mqttInited        = false;
 
// ---- Device announce: publish name/IP/MAC/fw to a shared feed ----
// Free AIO accounts allow 10 feeds and 30 publishes/min for the whole
// account, so all devices share ONE feed, with a random delay after connect
// (avoids a burst when the whole fleet reconnects) and an hourly refresh.
static const unsigned long ANNOUNCE_EVERY_MS = 3600000UL;
static unsigned long nextAnnounceMs = 0;
 
static void announceDevice(){
  JsonDocument d;
  d["type"]     = "onair-announce";   // lets you pick these out of a shared feed
  d["name"]     = deviceName;
  d["deviceId"] = aioDeviceId;
  d["ip"]       = WiFi.localIP().toString();
  d["mac"]      = macString();
  d["fw"]       = FIRMWARE_VERSION;
  String out; serializeJson(d, out);
  String topic = aioTopic(AIO_FEED_DEVICES);
  bool ok = mqtt.publish(topic.c_str(), out.c_str());
  Serial.printf("[AIO] Announce %s: %s\n", ok ? "sent" : "FAILED", out.c_str());
}
 
static void mqttBumpBackoff(){
  unsigned long next = mqttBackoffMs * 2UL;
  mqttBackoffMs = (next > MQTT_BACKOFF_MAX) ? MQTT_BACKOFF_MAX : next;
}
 
static void ensureMqtt(){
  unsigned long now = millis();
 
  if (!mqttInited){
    mqtt.setServer(AIO_BROKER, AIO_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(1024);   // default 256 is too small for AIO-wrapped JSON
    mqtt.setKeepAlive(30);
    mqtt.setSocketTimeout(10);
    mqttInited = true;
  }
 
  if (mqtt.connected()){
    // Only reset the backoff once the link has proven stable for a minute.
    if (mqttBackoffMs != MQTT_BACKOFF_MIN && (now - mqttConnectedAtMs) > 60000UL)
      mqttBackoffMs = MQTT_BACKOFF_MIN;
    if ((long)(now - nextAnnounceMs) >= 0){
      announceDevice();
      nextAnnounceMs = now + ANNOUNCE_EVERY_MS;
    }
    return;
  }
 
  // Connection was up and just dropped: log why, then wait before retrying.
  if (mqttWasConnected){
    mqttWasConnected = false;
    Serial.printf("[AIO] Dropped after %lus (state=%d)\n",
                  (now - mqttConnectedAtMs) / 1000UL, mqtt.state());
    nextMqttAttemptMs = now + mqttBackoffMs;
    mqttBumpBackoff();
    return;
  }
 
  if (WiFi.status() != WL_CONNECTED) return;
  if ((long)(now - nextMqttAttemptMs) < 0) return;
 
  String cid = mqttClientId();
  Serial.print(F("[AIO] Connecting MQTT as ")); Serial.print(cid); Serial.print(F("..."));
  if (mqtt.connect(cid.c_str(), AIO_USERNAME, AIO_KEY)){
    Serial.println(F("OK"));
    mqttWasConnected  = true;
    mqttConnectedAtMs = now;
    aioSubscribeAll();
    nextAnnounceMs = now + random(2000, 15000);   // jittered first announce
  } else {
    Serial.printf("FAIL rc=%d (next try in %lus)\n", mqtt.state(), mqttBackoffMs / 1000UL);
    nextMqttAttemptMs = now + mqttBackoffMs;
    mqttBumpBackoff();
  }
}
 
// Device ID / group edits no longer change what we subscribe to (targeting
// happens in the payload), so there is nothing to resubscribe. Instead this
// schedules a fresh announce so the device list reflects the new settings.
static void forceResubscribeNow() { nextAnnounceMs = millis() + 2000UL; }
 
// ---------------------- OTA ------------------------------------------
static void setupArduinoOTA(){
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA
    .onStart([](){ Serial.println("[OTA] Start"); })
    .onEnd([](){ Serial.println("\n[OTA] End"); })
    .onProgress([](unsigned int prog, unsigned int total){
      static uint32_t last=0; uint32_t pct = (prog*100)/total;
      if (pct!=last){ last=pct; Serial.printf("[OTA] %u%%\n", pct); }
    })
    .onError([](ota_error_t err){ Serial.printf("[OTA] Error %u\n", err); });
  ArduinoOTA.begin();
  Serial.println("[OTA] ArduinoOTA ready");
}
static void setupElegantOTA(){
  ElegantOTA.begin(&server);    // /update
  Serial.println("[OTA] ElegantOTA ready at /update");
}
static void startOtaRecoveryAp(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP("OnAirLight-OTA");
  Serial.printf("[OTA-RECOVERY] AP SSID: OnAirLight-OTA  IP: %s\n", WiFi.softAPIP().toString().c_str());
  startDiscoveryBlink();
  setupElegantOTA();
  server.on("/", [](){ server.sendHeader("Location","/update"); server.send(302); });
  // captive probes
  server.on("/favicon.ico", [](){ server.send(204); });
  server.on("/generate_204", [](){ server.send(204); });
  server.on("/hotspot-detect.html", [](){ server.send(204); });
  server.on("/connecttest.txt", [](){ server.send(204); });
  server.on("/ncsi.txt", [](){ server.send(204); });
  server.onNotFound([](){ server.sendHeader("Location","/update"); server.send(302); });
  server.begin();
}
 
// ---------------------- Web UI helpers (stream) -----------------------
static void beginStreamPage(const String& title){
  server.sendHeader("Connection","close");
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma","no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/html","");
  auto out = [&](const String&s){ server.sendContent(s); };
  out(F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>"));
  out(title);
  out(F("</title><style>"
        "body{font-family:sans-serif;background:#333;color:#fff;margin:0}"
        ".wrap{max-width:760px;margin:0 auto;padding:16px}"
        ".card{background:#444;border-radius:12px;padding:16px;margin:12px 0;box-shadow:0 2px 10px rgba(0,0,0,.2)}"
        "h1{text-align:center;margin:10px 0}"
        ".nav{display:flex;justify-content:center;gap:8px;margin-bottom:8px}"
        ".btn{display:inline-block;background:#0d6efd;color:#fff;border:none;border-radius:10px;padding:10px 16px;margin:6px 6px;text-decoration:none;cursor:pointer}"
        ".btn.sm{font-size:12px;padding:6px 10px;margin:4px 6px;opacity:.92}"
        ".off{background:#F44336}.available{background:#28a745}.away{background:#17a2b8}"
        "input[type=range]{width:200px}"
        "input[type=text],input[type=number]{padding:8px;border-radius:6px;border:1px solid #555;background:#222;color:#fff}"
        "input[type=color]{width:95px;height:32px;border:1px solid #777;background:transparent;border-radius:6px}"
        ".row{display:flex;align-items:center;gap:12px;flex-wrap:wrap}"
        ".kv{margin:8px 0} code{background:#222;padding:2px 6px;border-radius:6px}"
        "</style></head><body><div class='wrap'>"));
}
static void endStreamPage(){
  auto out = [&](const String&s){ server.sendContent(s); };
  out(F("<p style='text-align:center;color:#bbb;margin-top:16px'>Firmware v" FIRMWARE_VERSION " | MAC: "));
  out(macString());
  out(F("</p></div></body></html>"));
  server.sendContent("");
  server.client().stop();
}
static void cardStart(const __FlashStringHelper* t){ server.sendContent(F("<div class='card'><h3>")); server.sendContent(String(t)); server.sendContent(F("</h3>")); }
static void cardEnd(){ server.sendContent(F("</div>")); }
 
// ---------------------- Pages -----------------------------------------
static void handleRoot(){
  if (discoveryBlinkActive) stopDiscoveryBlink(true);
  Serial.println("[HTTP] GET /");
 
  beginStreamPage(deviceName);
 
  auto out = [&](const String&s){ server.sendContent(s); };
  out(F("<h1>")); out(deviceName); out(F("</h1>"));
  out(F("<div class='nav'><a class='btn sm' href='/advanced'>Advanced</a></div>"));
  out(F("<p style='text-align:center'>Current Mode: <b>"));
  out(getEffectName());
  out(F("</b></p>"));
 
  // Modes
  cardStart(F("Modes"));
  out(F("<a class='btn' href='/set?effect=solid'>Solid</a>"
        "<a class='btn' href='/set?effect=blink'>Blink</a>"
        "<a class='btn' href='/set?effect=party'>Party</a>"
        "<a class='btn' href='/set?effect=knight_rider'>Knight Rider</a>"
        "<a class='btn' href='/set?effect=breathe'>Breathe</a>"
        "<a class='btn' href='/set?effect=comet'>Comet</a>"
        "<a class='btn' href='/set?effect=rainbow'>Rainbow</a>"
        "<a class='btn' href='/set?effect=pulse'>Pulse</a>"
        "<a class='btn off' href='/set?effect=off'>OFF</a>"));
  cardEnd();
 
  // Presets
  cardStart(F("Presets"));
  out(F("<a class='btn off' href='/set?preset=busy'>Busy</a>"
        "<a class='btn available' href='/set?preset=available'>Available</a>"
        "<a class='btn away' href='/set?preset=away'>Away</a>"));
  cardEnd();
 
  // Color / Speed / Brightness
  cardStart(F("Color, Speed & Brightness"));
  int uiSpeed = (int)round( -log((double)effectDelay/200.0) / 0.035 ); uiSpeed = constrain(uiSpeed, 0, 100);
  out(F("<form action='/set' method='get'><div class='row'><div class='kv'>Color "
        "<input type='color' name='color' value='"));
  out(colorToHex(currentColor));
  out(F("'></div><div class='kv'>Speed "
        "<input type='range' name='speed' min='0' max='100' value='"));
  out(String(uiSpeed));
  out(F("'></div><div class='kv'>Brightness "
        "<input type='range' name='brightness' min='0' max='100' value='"));
  out(String(brightnessPctFrom255(globalBrightness)));
  out(F("'></div><div class='kv'><button class='btn' type='submit'>Apply</button></div></div></form>"));
  cardEnd();
 
  // AIO Device
  cardStart(F("Adafruit IO"));
  out(F("<form action='/save_aio_device' method='get' class='row' style='margin:0'>"
        "<div>Device ID</div><input type='text' name='id' maxlength='40' value='"));
  out(aioDeviceId);
  out(F("'><button class='btn' type='submit'>Save</button></form>"));
  cardEnd();
 
  // Groups
  cardStart(F("Groups"));
  out(F("<form action='/groups/add' method='get' class='row' style='margin:0 0 10px 0'>"
        "<input type='text' name='name' maxlength='32' placeholder='Add group (e.g. product_team)'>"
        "<button class='btn' type='submit'>Add</button></form>"));
  if (groups.empty()){
    out(F("<em>No groups configured.</em>"));
  } else {
    for (auto &g: groups){
      out(F("<div class='row' style='justify-content:space-between;border-bottom:1px solid #555;padding:6px 0;'>"
            "<div>"));
      out(g);
      out(F("</div><form action='/groups/del' method='get' style='margin:0'>"
            "<input type='hidden' name='name' value='"));
      out(g);
      out(F("'><button class='btn off' type='submit'>Remove</button></form></div>"));
    }
  }
  cardEnd();
 
  endStreamPage();
}
 
static void handleAdvanced(){
  if (discoveryBlinkActive) stopDiscoveryBlink(true);
  Serial.println("[HTTP] GET /advanced");
 
  beginStreamPage(String(deviceName) + " – Advanced");
  auto out = [&](const String&s){ server.sendContent(s); };
 
  out(F("<h1>Advanced</h1>"));
  out(F("<div class='nav'><a class='btn sm' href='/'>Back</a></div>"));
 
  // Hardware config
  cardStart(F("Hardware (LED Pin / Count / Gap)"));
  out(F("<form action='/save_hw' method='get' class='row' style='margin:0'>"
        "<div>Pin</div><input type='number' name='pin' min='0' max='21' value='"));
  out(String(gLedPin));
  out(F("'><div>Count</div><input type='number' name='count' min='1' max='600' value='"));
  out(String(gLedCount));
  out(F("'><div>Gap</div><input type='number' name='gap' min='0' max='1200' value='"));
  out(String(gLedGap));
  out(F("'>"
        "<button class='btn' type='submit'>Save & Reboot</button></form>"
        "<p style='color:#bbb;margin:8px 0 0 0'>Gap makes circular effects move smoothly over the physical gap.</p>"));
  cardEnd();
 
  // Last AIO Message
  cardStart(F("Last AIO Message"));
  out(F("<div class='kv'><b>Topic:</b> ")); out(lastMsgTopic);
  out(F("</div><div class='kv'><b>When:</b> "));
  if (lastMsgMs==0) out(F("—"));
  else {
    uint32_t ago = millis() - lastMsgMs;
    uint32_t s = ago/1000UL, m = s/60; s%=60;
    out(String(m) + "m" + String(s) + "s ago");
  }
  out(F("</div><div class='kv'><b>Payload:</b><br><code>"));
  out(lastMsgPayload.length()? lastMsgPayload : "-");
  out(F("</code></div>"));
  cardEnd();
 
  // Firmware/OTA
  cardStart(F("Firmware / OTA"));
  out(F("<div class='row'>"
        "<a class='btn sm' href='/update'>Open OTA Uploader</a>"
        "<a class='btn sm off' href='/otarecovery'>Enter OTA Recovery</a>"
        "</div>"));
  cardEnd();
 
  // System
  cardStart(F("System"));
  out(F("<form action='/rename' method='post' class='row' style='margin:0'>"
        "<input type='text' name='name' maxlength='40' value='"));
  out(deviceName);
  out(F("' placeholder='Device Name'>"
        "<button class='btn' type='submit'>Rename</button></form>"
        "<a class='btn' href='/reboot'>Reboot</a>"
        "<a class='btn off' href='/forgetwifi'>Forget Wi-Fi</a>"));
  cardEnd();
 
  endStreamPage();
}
 
// ------------- Web handlers (shared) ----------------------------------
static void applyPreset(const String& p){
  String m = p; m.trim(); m.toLowerCase();
  if (m=="available"||m=="away"||m=="busy") applyPresence(m);
}
static void handleSetEffect(){
  stopDiscoveryBlink(true);
  bool newPowerState = true;
  effectPosition=0; effectDirection=1;
 
  if (server.hasArg("preset")) {
    applyPreset(server.arg("preset"));
    powerState = true;
    server.sendHeader("Location","/"); server.send(302,"OK");
    return;
  }
 
  if (server.hasArg("effect")){
    String e=server.arg("effect"); e.toLowerCase();
    if      (e=="solid")         currentEffect=SOLID;
    else if (e=="blink")         currentEffect=BLINK;
    else if (e=="party")         currentEffect=PARTY;
    else if (e=="knight_rider")  currentEffect=KNIGHT_RIDER;
    else if (e=="breathe")       currentEffect=BREATHE;
    else if (e=="comet")         currentEffect=COMET;
    else if (e=="rainbow")       currentEffect=RAINBOW;
    else if (e=="pulse")         { currentEffect=PULSE; pulseResetRequested = true; }
    else if (e=="off")           { currentEffect=OFF; newPowerState=false; }
  }
  if (server.hasArg("speed")) {
    effectDelay = speedToDelay(server.arg("speed").toInt());
    if (currentEffect==PULSE) pulseResetRequested = true;
  }
  if (server.hasArg("brightness")) {
    const int pct = constrain(server.arg("brightness").toInt(), 0, 100);
    globalBrightness = brightness255FromPct(pct);
    if (powerState) {
      if (currentEffect == SOLID) setAllPixels(currentColor);
      else { strip.setBrightness(globalBrightness); strip.show(); if (currentEffect==PULSE) pulseResetRequested=true; }
    }
  }
  if (server.hasArg("color")){
    long cv=strtol(server.arg("color").c_str()+1,nullptr,16);
    uint8_t r=(cv>>16)&0xFF, g=(cv>>8)&0xFF, b=cv&0xFF;
    applyColorPreserveBrightness(r,g,b);
    if (powerState && currentEffect==SOLID) setAllPixels(currentColor);
    if (currentEffect==PULSE) pulseResetRequested = true;
  }
 
  if (!newPowerState || currentEffect==OFF) {
    powerState=false; setAllPixels(0);
  } else {
    powerState=true;
    if (currentEffect==SOLID) setAllPixels(currentColor);
  }
  server.sendHeader("Location","/"); server.send(302,"OK");
}
static void handleForgetWifi(){
  server.send(200,"text/plain","Forgetting Wi-Fi and rebooting...");
  File f = LittleFS.open(FORGET_FILE, "w"); if (f) { f.println("1"); f.close(); }
  wifiStore.clear();   // networks in secrets.h are still tried
  WiFi.mode(WIFI_STA); WiFi.persistent(true);
  wifiSetAutoConnect(false); WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, true);
  delay(100); ESP.restart();
}
static void handleReboot(){ server.send(200,"text/plain","Rebooting..."); delay(300); ESP.restart(); }
static void handleSaveAIODevice(){
  if (server.hasArg("id")){
    String v=cleanToken(server.arg("id")); if(!v.length()) v="unset"; aioDeviceId=v; saveAIODevice();
    forceResubscribeNow();
  }
  server.sendHeader("Location","/"); server.send(302);
}
static void handleGroupsAdd(){
  if (!server.hasArg("name")){ server.send(400,"text/plain","Missing name"); return; }
  String g=cleanToken(server.arg("name")); if(!g.length()){ server.send(400,"text/plain","Invalid group"); return; }
  for (auto &x:groups) if (eqId(x,g)){ server.sendHeader("Location","/"); server.send(302); return; }
  groups.push_back(g); saveGroups();
  forceResubscribeNow();
  server.sendHeader("Location","/"); server.send(302);
}
static void handleGroupsDel(){
  if (!server.hasArg("name")){ server.send(400,"text/plain","Missing name"); return; }
  String g=cleanToken(server.arg("name"));
  for (size_t i=0;i<groups.size();++i){ if (eqId(groups[i], g)){ groups.erase(groups.begin()+i); break; } }
  saveGroups();
  forceResubscribeNow();
  server.sendHeader("Location","/"); server.send(302);
}
static void handleOtaRecovery(){
  File f = LittleFS.open(OTAONLY_FILE, "w");
  if (f) { f.println("1"); f.close(); }
  server.send(200,"text/plain","Entering OTA Recovery… rebooting");
  delay(300); ESP.restart();
}
static void handleSaveHardware(){
  int pin   = server.hasArg("pin")   ? server.arg("pin").toInt()   : DEFAULT_LED_PIN;
  int count = server.hasArg("count") ? server.arg("count").toInt() : DEFAULT_LED_COUNT;
  int gap   = server.hasArg("gap")   ? server.arg("gap").toInt()   : DEFAULT_LED_GAP;
  pin   = constrain(pin, 0, 21);
  count = constrain(count, 1, 600);
  gap   = max(0, gap);
  gLedPin   = (uint8_t)pin;
  gLedCount = (uint16_t)count;
  gLedGap   = (uint16_t)gap;
  saveHardwareCfg();
  server.send(200,"text/plain","Saved. Rebooting to apply…");
  delay(300); ESP.restart();
}
 
// ---------------------- Buzzer (TDB05LFPN) ----------------------------
// beep(count,onMs,offMs) runs the pattern from an esp_timer callback.
static esp_timer_handle_t buzzTimer = nullptr;
static volatile uint8_t  buzzLeft  = 0;
static volatile uint16_t buzzOnMs  = 0, buzzOffMs = 0;
static volatile bool     buzzOn    = false;

static void buzzerWrite(bool on){ digitalWrite(BUZZER_PIN, on ? HIGH : LOW); buzzOn = on; }
static void buzzTimerCb(void*){
  if (buzzOn){
    buzzerWrite(false);
    if (buzzLeft) --buzzLeft;
    if (buzzLeft) esp_timer_start_once(buzzTimer, (uint64_t)buzzOffMs * 1000ULL);
  } else if (buzzLeft){
    buzzerWrite(true);
    esp_timer_start_once(buzzTimer, (uint64_t)buzzOnMs * 1000ULL);
  }
}
static void buzzerInit(){
  pinMode(BUZZER_PIN, OUTPUT); buzzerWrite(false);
  const esp_timer_create_args_t a = { .callback = &buzzTimerCb, .arg = nullptr,
                                      .dispatch_method = ESP_TIMER_TASK, .name = "buzz" };
  esp_timer_create(&a, &buzzTimer);
}
static void beep(uint8_t count, uint16_t onMs, uint16_t offMs){
  if (!count || !buzzTimer) return;
  esp_timer_stop(buzzTimer);
  buzzLeft = count; buzzOnMs = onMs; buzzOffMs = offMs;
  buzzerWrite(true);
  esp_timer_start_once(buzzTimer, (uint64_t)onMs * 1000ULL);
}

// ---------------------- Touch (HW-139) -------------------------------
// Gestures (a tap is any press+release; taps within TOUCH_GAP_MS chain together):
//   tap (any length, nothing after)  -> on/off; on = last mode used, or Party if none yet
//   1 tap, then hold                 -> cycle fun modes
//   2 taps, then hold                -> cycle presence: Available / Busy / Away
//   3 taps, then hold                -> fast comet cycling R/G/B/Y/C/M/White
// While holding, the light switches to the next item every TOUCH_STEP_MS;
// releasing keeps the one showing (confirm beep). Other tap counts are ignored.
enum TouchCycle : uint8_t { TC_NONE, TC_FUN, TC_PRESENCE, TC_COMET };

static const effect      FUN_EFFECTS[]  = { SOLID, BLINK, PARTY, KNIGHT_RIDER, BREATHE, COMET, RAINBOW, PULSE };
static const char* const PRESENCE[]     = { "available", "busy", "away" };
static const uint32_t    COMET_COLORS[] = { 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0x00FFFF, 0xFF00FF, 0xFFFFFF };
#define ARRAY_LEN(a) (int)(sizeof(a)/sizeof((a)[0]))

static effect     lastOnEffect  = OFF;     // OFF = no mode used yet since boot
static bool       touchStable   = false;   // debounced state
static bool       touchRaw      = false;
static uint32_t   touchRawMs    = 0;
static uint32_t   touchDownMs   = 0;
static uint32_t   touchUpMs     = 0;
static uint8_t    tapCount      = 0;       // completed taps in the current gesture
static TouchCycle touchCycle    = TC_NONE;
static int        cycleIdx      = 0;
static uint32_t   cycleNextMs   = 0;
static bool          cometFast  = false;   // effectDelay currently overridden by the 3-tap comet
static unsigned long savedDelay = 0;

static void restoreSpeed(){
  // Undo the fast-comet speed unless something else (web/AIO) changed it since.
  if (cometFast && effectDelay == speedToDelay(COMET_FAST_SPEED)) effectDelay = savedDelay;
  cometFast = false;
}
static void touchToggle(){
  stopDiscoveryBlink(true);
  if (powerState && currentEffect!=OFF){
    currentEffect = OFF; powerState = false; setAllPixels(0);
  } else {
    currentEffect = (lastOnEffect==OFF) ? PARTY : lastOnEffect;
    effectPosition = 0; effectDirection = 1;
    if (currentEffect==PULSE) pulseResetRequested = true;
    powerState = true;
    setAllPixels(currentEffect==SOLID ? currentColor : 0);
  }
  beep(1, BEEP_TAP_MS);
  Serial.printf("[TOUCH] tap -> %s\n", getEffectName().c_str());
}
static int cycleLen(TouchCycle c){
  switch (c){
    case TC_FUN:      return ARRAY_LEN(FUN_EFFECTS);
    case TC_PRESENCE: return ARRAY_LEN(PRESENCE);
    case TC_COMET:    return ARRAY_LEN(COMET_COLORS);
    default:          return 1;
  }
}
// Applies item i of the cycle live, so the light shows exactly what release will keep.
static void cycleApply(TouchCycle c, int i){
  stopDiscoveryBlink(true);
  if (c==TC_FUN){
    restoreSpeed();
    currentEffect = FUN_EFFECTS[i];
    effectPosition = 0; effectDirection = 1;
    if (currentEffect==PULSE) pulseResetRequested = true;
    powerState = true;
    setAllPixels(currentEffect==SOLID ? currentColor : 0);
  } else if (c==TC_PRESENCE){
    restoreSpeed();
    applyPresence(PRESENCE[i]);
  } else if (c==TC_COMET){
    if (!cometFast){ savedDelay = effectDelay; cometFast = true; }
    effectDelay = speedToDelay(COMET_FAST_SPEED);
    const uint32_t col = COMET_COLORS[i];
    applyColorPreserveBrightness((col>>16)&0xFF, (col>>8)&0xFF, col&0xFF);
    if (currentEffect!=COMET || !powerState) setAllPixels(0);
    currentEffect = COMET; powerState = true;
  }
}
static void serviceTouch(){
  const uint32_t now = millis();
  // Remember the last "on" mode from any source (web, AIO, touch) for tap-to-restore.
  if (powerState && currentEffect!=OFF) lastOnEffect = currentEffect;

  // Debounce: accept a new level only after it has been stable for TOUCH_DEBOUNCE_MS.
  const bool raw = (digitalRead(TOUCH_PIN) == TOUCH_ACTIVE);
  if (raw != touchRaw){ touchRaw = raw; touchRawMs = now; }
  if (raw != touchStable && (now - touchRawMs) >= TOUCH_DEBOUNCE_MS){
    touchStable = raw;
    if (touchStable){
      touchDownMs = now;
    } else if (touchCycle != TC_NONE){            // release ends a cycle: keep it
      beep(1, BEEP_TAP_MS);
      Serial.printf("[TOUCH] cycle %d -> item %d (%s)\n", (int)touchCycle, cycleIdx, getEffectName().c_str());
      touchCycle = TC_NONE; tapCount = 0;
    } else {
      if (tapCount < 255) ++tapCount;
      touchUpMs = now;
    }
  }

  if (touchStable){
    const uint32_t held = now - touchDownMs;
    if (touchCycle == TC_NONE && tapCount >= 1 && tapCount <= 3 && held >= TOUCH_HOLD_MS){
      touchCycle = (TouchCycle)tapCount;           // 1 = fun, 2 = presence, 3 = comet
      cycleIdx = 0;
      cycleApply(touchCycle, cycleIdx);
      cycleNextMs = now + TOUCH_STEP_MS;
    } else if (touchCycle != TC_NONE && (long)(now - cycleNextMs) >= 0){
      cycleIdx = (cycleIdx + 1) % cycleLen(touchCycle);
      cycleApply(touchCycle, cycleIdx);
      cycleNextMs += TOUCH_STEP_MS;
    }
  } else if (tapCount && (now - touchUpMs) >= TOUCH_GAP_MS){
    if (tapCount == 1) touchToggle();
    else Serial.printf("[TOUCH] %u taps without hold - ignored\n", (unsigned)tapCount);
    tapCount = 0;
  }
}

// ---------------------- Setup / Loop ---------------------------------
void setup(){
  buzzerInit();   // buzzer silent ASAP
  delay(200);
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && (millis()-t0)<2500) delay(10);
  Serial.println("\n[BOOT] On Air Light v" FIRMWARE_VERSION " (ESP32-C3)");
 
  if (LittleFS.begin(true)){
    loadConfig();
    loadAIODevice();
    loadGroups();
    loadHardwareCfg();
    loadLastMsg();
    wifiStore.load();
  } else {
    Serial.println("[BOOT] LittleFS mount FAILED");
  }
 
  initStrip();
  // Pulldown keeps the pin LOW if the module is unplugged (module drives it when present)
  pinMode(TOUCH_PIN, INPUT_PULLDOWN);
  currentColor = RGB(255,0,0);
  setAllPixels(0);
  float tempV; rgbToHsv(currentColor, targetHue, targetSaturation, tempV);
 
  const bool otaOnly = LittleFS.exists(OTAONLY_FILE);
  if (otaOnly) { LittleFS.remove(OTAONLY_FILE); startOtaRecoveryAp(); return; }
 
  bool forgetOnce = LittleFS.exists(FORGET_FILE);
  if (forgetOnce) { LittleFS.remove(FORGET_FILE); }
 
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  wifiSetAutoConnect(!forgetOnce);
  WiFi.setAutoReconnect(!forgetOnce);
  WiFi.setSleep(false);
 
  if (forgetOnce) startConfigPortalOnly(); else connectWithFallbacks();
  effectDelay = speedToDelay(50);
 
  ensureMqtt();
 
  // OTA
  setupArduinoOTA();
  setupElegantOTA();
 
  // Routes
  server.on("/", handleRoot);
  server.on("/advanced", handleAdvanced);
  server.on("/set", handleSetEffect);
  server.on("/rename", HTTP_POST, [](){
    if (server.hasArg("name")){
      strncpy(deviceName, server.arg("name").c_str(), sizeof(deviceName));
      deviceName[sizeof(deviceName)-1]='\0'; saveConfig();
      forceResubscribeNow();
    }
    server.sendHeader("Location","/advanced"); server.send(302);
  });
  server.on("/forgetwifi", handleForgetWifi);
  server.on("/reboot", handleReboot);
  server.on("/save_aio_device", handleSaveAIODevice);
  server.on("/groups/add", handleGroupsAdd);
  server.on("/groups/del", handleGroupsDel);
  server.on("/otarecovery", handleOtaRecovery);
  server.on("/save_hw", handleSaveHardware);
 
  // health + captive probes
  server.on("/health", [](){ server.send(200,"text/plain","OK"); });
  server.on("/favicon.ico", [](){ server.send(204); });
  server.on("/generate_204", [](){ server.send(204); });
  server.on("/hotspot-detect.html", [](){ server.send(204); });
  server.on("/connecttest.txt", [](){ server.send(204); });
  server.on("/ncsi.txt", [](){ server.send(204); });
 
  server.onNotFound([](){ handleRoot(); });
  server.begin();
 
  Serial.printf("[BOOT] Wi-Fi:%s  IP:%s  MQTT:%s  MAC:%s  LED(pin=%u,count=%u,gap=%u)\n",
    (WiFi.status()==WL_CONNECTED?"OK":"DOWN"),
    WiFi.localIP().toString().c_str(),
    mqtt.connected()?"OK":"DOWN",
    macString().c_str(),
    (unsigned)gLedPin, (unsigned)gLedCount, (unsigned)gLedGap);
}
 
void loop(){
  server.handleClient();
  serviceWifi();
  ensureMqtt();
  mqtt.loop();
  ArduinoOTA.handle();
  serviceTouch();
 
  if (!statusActive){
    switch (currentEffect){
      case BLINK:        handleBlink(); break;
      case PARTY:        handleParty(); break;
      case KNIGHT_RIDER: handleKnightRider(); break;
      case BREATHE:      handleBreathe(); break;
      case COMET:        handleComet(); break;
      case RAINBOW:      handleRainbow(); break;
      case PULSE:        handlePulse(); break;
      default: break;
    }
  }
  serviceBlink();
 
  if (apRunning && (long)(millis() - apOffAtMs) >= 0) {
    stopDiscoveryBlink(false);
  }
 
  if (millis()-lastHeartbeatMs >= 10000){
    lastHeartbeatMs=millis();
    Serial.printf("[HB] Uptime:%lus | Wi-Fi:%s | IP:%s | MQTT:%s | Groups:%u | MAC:%s | Bright:%u/255 (%d%%)\n",
      millis()/1000UL,
      (WiFi.status()==WL_CONNECTED)?"OK":"DOWN",
      WiFi.localIP().toString().c_str(),
      mqtt.connected()?"OK":"DOWN",
      (unsigned)groups.size(),
      macString().c_str(),
      (unsigned)globalBrightness,
      brightnessPctFrom255(globalBrightness));
  }
}