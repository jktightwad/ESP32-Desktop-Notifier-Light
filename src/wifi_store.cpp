#include "wifi_store.h"
#include <LittleFS.h>

namespace { const char* const PATH = "/wifi.txt"; }

void WifiStore::load() {
  nets_.clear();
  if (!LittleFS.exists(PATH)) return;
  File f = LittleFS.open(PATH, "r");
  if (!f) return;
  while (f.available() && nets_.size() < MAX_NETWORKS) {
    String line = f.readStringUntil('\n');
    if (line.endsWith("\r")) line.remove(line.length() - 1);
    int tab = line.indexOf('\t');
    if (tab <= 0) continue;
    nets_.push_back({line.substring(0, tab), line.substring(tab + 1)});
  }
  f.close();
}

void WifiStore::save() const {
  File f = LittleFS.open(PATH, "w");
  if (!f) { Serial.println("Wi-Fi list: could not write file"); return; }
  for (const WifiCred& n : nets_) f.printf("%s\t%s\n", n.ssid.c_str(), n.pass.c_str());
  f.close();
}

void WifiStore::remember(const String& ssid, const String& pass) {
  if (!ssid.length()) return;
  if (!nets_.empty() && nets_[0].ssid == ssid && nets_[0].pass == pass) return;   // already first
  for (size_t i = 0; i < nets_.size(); i++) {
    if (nets_[i].ssid == ssid) { nets_.erase(nets_.begin() + i); break; }
  }
  nets_.insert(nets_.begin(), {ssid, pass});
  if (nets_.size() > MAX_NETWORKS) nets_.resize(MAX_NETWORKS);
  save();
  Serial.printf("Wi-Fi list: saved \"%s\" (%u networks)\n", ssid.c_str(), (unsigned)nets_.size());
}

void WifiStore::clear() {
  nets_.clear();
  LittleFS.remove(PATH);
  Serial.println("Wi-Fi list: cleared");
}
