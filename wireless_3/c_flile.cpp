#include "WiFi.h"
#include <WiFiUdp.h>
#include <cmath>
#include <cstdio> // For snprintf

// --- Configuration ---
const char* const SSID = "laptop";
const char* const PASSWORD = "pppppppp";

const char* const COMPUTER_IP = "192.168.137.1";
const int UDP_PORT = 4210;

// Path Loss Model Constants
// TX_POWER is the RSSI measured at 1 meter (A).
const double TX_POWER = -43.5;
const double PATH_LOSS_EXPONENT = 3.7;

// Kalman Filter Constants
const double KALMAN_PROCESS_NOISE = 0.125;
const double KALMAN_MEASUREMENT_NOISE = 4.0;

// --- Data Structures ---

// Encapsulates the state for one AP and its Kalman filter
struct AccessPoint {
  const char* ssid;
  double kalman_estimate;
  double kalman_error_estimate;
  bool found_in_last_scan;
};

// Define all target APs
AccessPoint target_aps[] = {
  {"narzo", -60.0, 1.0, false},
  {"laptop", -60.0, 1.0, false},
  {"A", -60.0, 1.0, false}
};

constexpr size_t NUM_APS = sizeof(target_aps) / sizeof(target_aps[0]);

// --- UDP object ---
WiFiUDP udp;

// --- Function declarations ---
double kalman_update(AccessPoint& ap, double measurement);
double calculate_distance(double rssi);
void send_data_udp(const char* data);
void start_next_scan();
void process_scan_results(int num_networks);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 WiFi Trilateration (Async Scan)");

  // 1. Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // 2. Start UDP
  udp.begin(UDP_PORT);

  // 3. Start the first asynchronous scan
  start_next_scan();
}

void loop() {
  // Check if the asynchronous scan is complete
  int n = WiFi.scanComplete();

  if (n > 0) {
    process_scan_results(n);
    WiFi.scanDelete(); // Clean up results
    start_next_scan(); // Start the next scan immediately
  } else if (n == WIFI_SCAN_FAILED) {
    Serial.println("Scan failed. Retrying...");
    start_next_scan();
  } else {
    // n == WIFI_SCAN_RUNNING or n == WIFI_SCAN_DONE (if already processed)
    // The ESP32 is free to do other tasks here if needed
    // We add a small delay to avoid hammering the `scanComplete` check
    delay(50);
  }
}

// --- Helper Functions ---

void start_next_scan() {
  Serial.println("\nStarting Async Scan...");
  // Asynchronous scan (true), hidden SSIDs (false), passive (true), 80ms/channel
  int result = WiFi.scanNetworks(true, false, true, 80);
  if (result == WIFI_SCAN_RUNNING) {
    Serial.println("Scan is running in background.");
  }
}

void process_scan_results(int num_networks) {
  Serial.printf("Scan complete. Found %d networks.\n", num_networks);

  // 1. Reset 'found' flag for all target APs
  for (size_t i = 0; i < NUM_APS; i++) {
    target_aps[i].found_in_last_scan = false;
  }

  // 2. Iterate over the scan results and update Kalman estimates
  for (int i = 0; i < num_networks; ++i) {
    String scanned_ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);

    for (size_t j = 0; j < NUM_APS; j++) {
      if (scanned_ssid.equals(target_aps[j].ssid)) {
        target_aps[j].kalman_estimate = kalman_update(target_aps[j], rssi);
        target_aps[j].found_in_last_scan = true;
        break;
      }
    }
  }

  // 3. Prepare and Send UDP Data
  // Use a char buffer for efficient string formatting
  // Need space for "DATA:" + (3 * "xxx.xxx" + 2 * ",") + NULL
  char data_buffer[NUM_APS * 8 + 10] = "DATA:";
  size_t current_len = 5;

  for (size_t i = 0; i < NUM_APS; i++) {
    double dist;

    if (target_aps[i].found_in_last_scan) {
      dist = calculate_distance(target_aps[i].kalman_estimate);
    } else {
      // AP was not found. Use -1.0 as an invalid sentinel value
      // This is robust for the PC-side trilateration solver.
      dist = -1.0;
      Serial.printf("AP '%s' not found.\n", target_aps[i].ssid);
    }

    // Format distance (3 decimal places) into the buffer
    // snprintf returns the number of characters that would have been written
    int written = snprintf(
      data_buffer + current_len,
      sizeof(data_buffer) - current_len,
      "%.3f",
      dist
    );
    current_len += written;

    if (i < NUM_APS - 1) {
      // Add comma separator
      snprintf(data_buffer + current_len, sizeof(data_buffer) - current_len, ",");
      current_len += 1;
    }
  }

  Serial.println(data_buffer);
  send_data_udp(data_buffer);
}

// The Kalman filter function now takes a reference to the struct
double kalman_update(AccessPoint& ap, double measurement) {
  // 1. Prediction (Time Update) - Error increases
  ap.kalman_error_estimate += KALMAN_PROCESS_NOISE;

  // 2. Update (Measurement Update)
  double gain = ap.kalman_error_estimate /
                (ap.kalman_error_estimate + KALMAN_MEASUREMENT_NOISE);
  ap.kalman_estimate += gain * (measurement - ap.kalman_estimate);
  ap.kalman_error_estimate *= (1.0 - gain);

  return ap.kalman_estimate;
}

// Distance calculation remains the same
double calculate_distance(double rssi) {
  // Log-Distance Path Loss Model
  // d = 10^((A - RSSI) / (10 * n))
  double exponent = (TX_POWER - rssi) / (10.0 * PATH_LOSS_EXPONENT);
  return pow(10.0, exponent);
}

// The UDP function now takes a C-style string (char*)
void send_data_udp(const char* data) {
  udp.beginPacket(COMPUTER_IP, UDP_PORT);
  udp.print(data);
  udp.endPacket();
}
