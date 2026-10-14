// Copy this file to secrets.h (same folder) and fill in your values.
// secrets.h is excluded from git by .gitignore.
#pragma once

// Wi-Fi networks to try, in addition to the ones the device has joined before
// (every network it connects to, including through the OnAirLight-Setup portal,
// is saved on the device). The strongest one in range is used.
#define WIFI_NETWORKS { \
    {"office-network", "office-password"}, \
    {"home-network",   "home-password"}, \
}

static const char* AIO_USERNAME      = "your-adafruit-io-username";
static const char* AIO_KEY           = "your-adafruit-io-key";
