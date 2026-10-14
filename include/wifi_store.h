#pragma once
#include <Arduino.h>
#include <vector>

// Wi-Fi networks the board has joined, saved in /wifi.txt on the file system
// (one "ssid<TAB>password" per line, most recently used first). Survives
// reboots and firmware uploads; replaced only by "Upload Filesystem Image".
// Note: like the ESP32's own Wi-Fi storage, passwords are stored unencrypted
// in flash, so anyone with the board and a USB cable could read them.
struct WifiCred {
  String ssid;
  String pass;
};

class WifiStore {
 public:
  static const size_t MAX_NETWORKS = 10;

  void load();
  const std::vector<WifiCred>& networks() const { return nets_; }

  // Adds or updates a network and moves it to the front (most recent).
  // Saves only if something changed.
  void remember(const String& ssid, const String& pass);

  // Forgets every saved network (deletes the file).
  void clear();

 private:
  void save() const;
  std::vector<WifiCred> nets_;
};
