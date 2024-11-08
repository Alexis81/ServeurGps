#include <WiFi.h>
#include <HardwareSerial.h>
#include "Arduino.h"

// Configuration du point d'accès WiFi
const char* ssid = "GPS_Simulator";
const char* password = "gps12345";
const int serverPort = 8080;

// Configuration GPS
HardwareSerial GPSSerial(1);
const int GPS_TX_PIN = 17;
const int GPS_RX_PIN = 16;

// Configuration du timeout client
const unsigned long CLIENT_TIMEOUT = 5000; // 5 secondes
unsigned long lastClientActivity = 0;

WiFiServer server(serverPort);
WiFiClient client;

// Buffer pour stocker les données GPS
char gpsBuffer[1024];
int bufferIndex = 0;

// Structure pour stocker les données GPS courantes
struct {
    float latitude;
    float longitude;
    float speed;
    int satellites;
    int precision_cm;
    bool hasValidFix;
    int hours;
    int minutes;
    int seconds;
    unsigned long lastStatsTime;
} gpsData = {0};

// Commandes UBX pour configurer le GPS
const unsigned char UBLOX_INIT[] PROGMEM = {
    // CFG-RATE pour 10Hz (100ms)
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
    
    // CFG-PRT (Configuration du port série à 115200 bauds)
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E,
    
    // CFG-MSG (Configuration des messages NMEA - réduire les messages inutiles)
    // Désactiver GLL
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,
    // Désactiver GSA
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,
    // Désactiver GSV
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,
    // Désactiver VTG
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46
};

void configureGPS() {
    Serial.println("Configuration du GPS...");
    
    // Envoi des commandes de configuration
    for(unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {
        GPSSerial.write(pgm_read_byte(UBLOX_INIT + i));
    }
    
    // Attendre que la configuration soit appliquée
    delay(1000);
    
    Serial.println("GPS configuré !");
}

void checkClientTimeout() {
    if (client && client.connected()) {
        if (millis() - lastClientActivity > CLIENT_TIMEOUT) {
            Serial.println("Client timeout - Disconnecting");
            client.stop();
        }
    }
}

bool isClientConnected() {
    if (!client) return false;
    
    if (!client.connected()) {
        Serial.println("Client disconnected - Cleaning up");
        client.stop();
        return false;
    }
    
    return true;
}

void handleNewClient() {
    if (server.hasClient()) {
        if (!isClientConnected()) {
            // Accepte le nouveau client
            client = server.available();
            lastClientActivity = millis();
            Serial.printf("New client connected from IP: %s\n", client.remoteIP().toString().c_str());
        } else {
            // Rejette proprement le nouveau client
            WiFiClient rejectClient = server.available();
            rejectClient.println("Server busy - Try again later");
            rejectClient.stop();
            Serial.println("Client connection rejected - Server busy");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=== GPS Server Starting ===");
    
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // Configuration du GPS
    configureGPS();
    
    // Changement de la vitesse du port série GPS à 115200 bauds
    GPSSerial.flush();
    GPSSerial.updateBaudRate(115200);
    
    Serial.printf("GPS Serial initialized on pins TX:%d, RX:%d\n", GPS_TX_PIN, GPS_RX_PIN);
    
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("AP Started - SSID: %s\n", ssid);
    Serial.printf("AP IP address: %s\n", myIP.toString().c_str());
    
    server.begin();
    Serial.printf("TCP Server started on port %d\n", serverPort);
    Serial.println("=== Initialization Complete ===\n");
}

// Fonction pour extraire les données de la trame GNRMC
void parseGNRMC(const char* frame) {
    char time_str[16] = {0};
    char status;
    char lat_str[16] = {0};
    char lat_dir;
    char lon_str[16] = {0};
    char lon_dir;
    char speed_str[16] = {0};

    if(sscanf(frame, "$GNRMC,%[^,],%c,%[^,],%c,%[^,],%c,%[^,]",
              time_str, &status, lat_str, &lat_dir, lon_str, &lon_dir, speed_str) == 7) {
        
        if(status == 'A') { // Si fix valide
            // Parse time
            if(strlen(time_str) >= 6) {
                gpsData.hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
                gpsData.minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
                gpsData.seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');
            }

            // Parse latitude
            float lat_deg = (lat_str[0] - '0') * 10 + (lat_str[1] - '0');
            float lat_min = atof(lat_str + 2);
            gpsData.latitude = lat_deg + lat_min / 60.0;
            if(lat_dir == 'S') gpsData.latitude = -gpsData.latitude;

            // Parse longitude
            float lon_deg = (lon_str[0] - '0') * 100 + (lon_str[1] - '0') * 10 + (lon_str[2] - '0');
            float lon_min = atof(lon_str + 3);
            gpsData.longitude = lon_deg + lon_min / 60.0;
            if(lon_dir == 'W') gpsData.longitude = -gpsData.longitude;

            // Parse vitesse (conversion en km/h)
            gpsData.speed = atof(speed_str) * 1.852;
            gpsData.hasValidFix = true;

            Serial.printf("Time parsed: %02d:%02d:%02d\n", 
                         gpsData.hours, gpsData.minutes, gpsData.seconds);
        } else {
            gpsData.hasValidFix = false;
        }
    }
}

// Fonction pour parser la trame GNGGA
void parseGNGGA(const char* frame) {
    char dummy[16];
    int fix = 0;
    int sats = 0;
    float hdop = 0.0;
    
    if (sscanf(frame, "$GNGGA,%[^,],%[^,],%[^,],%[^,],%[^,],%d,%d,%f",
               dummy, dummy, dummy, dummy, dummy, &fix, &sats, &hdop) >= 8) {
        
        gpsData.satellites = sats;
        
        // Calcul de la précision en cm
        int basePrecision;
        switch(fix) {
            case 4: basePrecision = 1; break;   // RTK fix
            case 5: basePrecision = 10; break;  // RTK float
            case 2: basePrecision = 100; break; // DGPS
            case 1: basePrecision = 200; break; // GPS standard
            default: basePrecision = 500;       // Non valide
        }
        gpsData.precision_cm = (int)(basePrecision * hdop);
    }
}

// Fonction pour envoyer les données minimales
void sendMinimalData() {
    if (client && client.connected() && gpsData.hasValidFix) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), 
            "%.6f,%.6f,%.1f,%d,%d,%02d:%02d:%02d\n", 
            gpsData.latitude, 
            gpsData.longitude, 
            gpsData.speed, 
            gpsData.satellites, 
            gpsData.precision_cm,
            gpsData.hours,
            gpsData.minutes,
            gpsData.seconds);
            
        client.write((uint8_t*)buffer, len);
        Serial.printf("Data sent: %s", buffer);
    }
}

void loop() {
    // Vérifie le timeout du client
    checkClientTimeout();
    
    // Gère les nouvelles connexions
    handleNewClient();

    // Si un client est connecté, traite les données GPS
    if (isClientConnected()) {
        lastClientActivity = millis(); // Met à jour le timestamp d'activité
        
        while (GPSSerial.available()) {
            char c = GPSSerial.read();
            gpsBuffer[bufferIndex++] = c;
            
            if (c == '\n' || bufferIndex >= sizeof(gpsBuffer)-1) {
                gpsBuffer[bufferIndex] = '\0';
                
                if (strncmp(gpsBuffer, "$GNRMC", 6) == 0) {
                    parseGNRMC(gpsBuffer);
                }
                else if (strncmp(gpsBuffer, "$GNGGA", 6) == 0) {
                    parseGNGGA(gpsBuffer);
                    // Envoyer les données après avoir reçu la trame GGA
                    sendMinimalData();
                }
                
                bufferIndex = 0;
            }
            
            if (bufferIndex >= sizeof(gpsBuffer)) {
                bufferIndex = 0;
                Serial.println("Buffer overflow prevented");
            }
        }
    } else {
        // Vide le buffer GPS si pas de client
        while (GPSSerial.available()) {
            GPSSerial.read();
        }
        bufferIndex = 0;
    }

    // Statistiques périodiques
    unsigned long currentTime = millis();
    if (currentTime - gpsData.lastStatsTime >= 5000) {
        Serial.println("\n=== Stats ===");
        Serial.printf("Fix valid: %s\n", gpsData.hasValidFix ? "Yes" : "No");
        if(gpsData.hasValidFix) {
            Serial.printf("Position: %.6f, %.6f\n", gpsData.latitude, gpsData.longitude);
            Serial.printf("Speed: %.1f km/h\n", gpsData.speed);
            Serial.printf("Satellites: %d\n", gpsData.satellites);
            Serial.printf("Precision: %d cm\n", gpsData.precision_cm);
        }
        Serial.printf("Client connected: %s\n", client ? "Yes" : "No");
        Serial.println("============\n");
        gpsData.lastStatsTime = currentTime;
    }
    
    delay(1);
}