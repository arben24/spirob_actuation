#include <Arduino.h>
#include <Wire.h>
#include "ForceSensorAnu78025.h"

// --- Hardware Einstellungen ---
#define I2C_MULTIPLEXER_ADDR 0x70  // Adresse des TCA9548A Multiplexers
#define ANU78025_I2C_ADDR    0x2A  // Standardadresse NAU7802
#define MULTIPLEXER_CHANNEL  7     // Kanal am Multiplexer

// --- Kalibrierung (Hier Werte eintragen nach erfolgreicher Kalibrierung) ---
//#define OFFSET 153859L    // Beispiel: Deinen Offset hier einkommentieren
//#define SCALE  105.13830   // Beispiel: Deinen Scale hier einkommentieren

ForceSensorAnu78025* sensor;

// Zustände für die Kalibrierung
enum AppState {
    IDLE,               // Normaler Messbetrieb
    CALIB_WAIT_WEIGHT,  // Warte, bis Nutzer Gewicht aufgelegt hat
    CALIB_WAIT_INPUT    // Warte auf Eingabe der Masse via Serial
};

AppState currentState = IDLE;
long calibrationRawValue = 0;  // Speichert den Raw-Wert während Kalibrierung

void setup() {
    Serial.begin(115200);
    delay(1000); // Warten auf Serial Monitor
    Wire.begin(); // I2C Bus starten

    Serial.println("\n=== ANU78025 ROBUST TEST + KALIBRIERUNG ===");
    Serial.println("Befehle:");
    Serial.println("  't' -> Tare (Null setzen)");
    Serial.println("  'c' -> Kalibrierung starten");
    
    // Sensor Objekt erstellen (I2C Adr, Mux Channel)
    sensor = new ForceSensorAnu78025(ANU78025_I2C_ADDR, MULTIPLEXER_CHANNEL);

    // --- Vordefinierte Kalibrierung anwenden (falls definiert) ---
    #if defined(OFFSET) && defined(SCALE)
        sensor->setCalibration(OFFSET, SCALE);
        Serial.print("Vorgegebener Offset gesetzt: "); Serial.println(OFFSET);
        Serial.print("Vorgegebener Scale-Faktor gesetzt: "); Serial.println(SCALE);
    #elif defined(OFFSET)
        sensor->setOffset(OFFSET);
        Serial.print("Vorgegebener Offset gesetzt: "); Serial.println(OFFSET);
    #elif defined(SCALE)
        sensor->setScale(SCALE);
        Serial.print("Vorgegebener Scale-Faktor gesetzt: "); Serial.println(SCALE);
    #endif

    // --- Initialisierung ---
    Serial.print("Initialisiere ANU78025 auf Channel "); 
    Serial.print(MULTIPLEXER_CHANNEL);
    Serial.println("...");

    int retryCount = 0;
    while(sensor->begin() == false) {
        Serial.print(".");
        delay(500);
        retryCount++;
        if(retryCount > 10) {
            Serial.println("\nFEHLER: Sensor antwortet nicht! (I2C/Multiplexer prüfen)");
            while(1) delay(1000); // Stop
        }
    }
    Serial.println("\n✓ ANU78025 init OK");

    // Wenn kein Offset definiert ist, machen wir Tare beim Start
    #ifndef OFFSET
        Serial.println("Kein Offset vorgegeben -> Fuehre Tare durch...");
        sensor->tare();
        Serial.println("✓ Tare init OK");
    #endif
}

void loop() {
    // ---------------------------------------------------------
    // 1. ZENTRALES SENSOR UPDATE
    // ---------------------------------------------------------
    sensor->update(); // Muss zyklisch laufen

    // ---------------------------------------------------------
    // 2. SERIAL INPUT VERARBEITUNG (STATE MACHINE)
    // ---------------------------------------------------------
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Entfernt \r \n und Leerzeichen

        if (input.length() > 0) {
            Serial.print("Cmd: "); Serial.println(input);

            // --- Globale Befehle ---
            if (input == "t") {
                sensor->tare();
                Serial.println("✓ Tare ausgeführt.");
                currentState = IDLE; // Reset State
                return;
            }

            // --- State Machine ---
            switch (currentState) {
                case IDLE:
                    if (input == "c") {
                        currentState = CALIB_WAIT_WEIGHT;
                        Serial.println("\n--- KALIBRIERUNG GESTARTET ---");
                        Serial.println("1. Lege eine bekannte Masse auf.");
                        Serial.println("2. Warte kurz bis stabil.");
                        Serial.println("3. Sende 's' um den Wert zu nehmen.");
                    } else {
                        Serial.println("Unbekannt. 'c' für Kalibrierung, 't' für Tare.");
                    }
                    break;

                case CALIB_WAIT_WEIGHT:
                    if (input == "s") {
                        // Wir lesen den aktuellen Rohwert direkt aus der Update-Schleife
                        // (oder rufen readRaw() auf, falls Update nicht schnell genug war)
                        calibrationRawValue = sensor->readRaw();
                        
                        currentState = CALIB_WAIT_INPUT;
                        Serial.printf("\n✓ Raw-Wert gespeichert: %ld\n", calibrationRawValue);
                        Serial.println("4. Gib jetzt die Masse in GRAMM ein (z.B. 1000):");
                    } else {
                        Serial.println("Bitte 's' senden, wenn Gewicht stabil liegt.");
                    }
                    break;

                case CALIB_WAIT_INPUT:
                    float mass_g = input.toFloat();
                    if (mass_g > 0) {
                        // Berechnung des Scale Factors:
                        // Scale = (Raw - Offset) / Masse
                        // Wichtig: Wir brauchen den Offset zum Zeitpunkt der Messung
                        long offset = sensor->getOffset();
                        float rawDelta = (float)(calibrationRawValue - offset);
                        
                        // Wir kalibrieren so, dass getWeight() später GRAMM ausgibt (oder kg, je nach Wunsch)
                        // Wenn wir hier durch Gramm teilen, ist Scale = "Raw pro Gramm".
                        // getWeight() liefert dann Gramm.
                        float newScale = rawDelta / mass_g;
                        
                        Serial.println("\n🎉 KALIBRIERUNG FERTIG!");
                        Serial.printf("  Masse: %.1f g\n", mass_g);
                        Serial.printf("  Raw:   %ld\n", calibrationRawValue);
                        Serial.printf("  Offset: %ld\n", offset);
                        Serial.printf("  Scale: %.5f\n", newScale);
                        Serial.println("-------------------------------------");
                        Serial.printf("  Code: #define SCALE %.5f\n", newScale);
                        Serial.printf("  Code: #define OFFSET %ldL\n", offset);
                        Serial.println("-------------------------------------");
                        
                        sensor->setScale(newScale);
                        currentState = IDLE;
                    } else {
                        Serial.println("Ungültige Eingabe! Bitte positive Zahl (Gramm) eingeben:");
                    }
                    break;
            }
        }
    }

    // ---------------------------------------------------------
    // 3. AUSGABE (Rate Limited)
    // ---------------------------------------------------------
    static unsigned long lastPrint = 0;
    // Ausgabe alle 250ms (nicht 1ms, das ist zu schnell für Serial!)
    if (millis() - lastPrint > 1000) { 
        lastPrint = millis();
        
        if (currentState == IDLE || currentState == CALIB_WAIT_WEIGHT) {
            long raw = sensor->readRaw();
            float weight = sensor->getWeight(); // In der Einheit der Kalibrierung (z.B. Gramm)
            float force = sensor->getForce();   // In Newton (sofern getWeight kg/g liefert und Logik stimmt)
            
            // Wenn Kalibrierung auf Gramm war, ist "force" evtl falsch, 
            // da getForce() in der Lib oft *9.81 rechnet unter Annahme, dass Weight = KG ist.
            // Falls du in Gramm kalibrierst, ist force um Faktor 1000 zu hoch.
            // Besser: Kalibriere in KG (siehe Kommentar unten).
            
            Serial.printf("State:%-8s | Raw:%8ld | Weight:%8.2f | Force:%8.3f N\n", 
                (currentState == IDLE) ? "RUN" : "CALIB", 
                raw, weight, force);
        }
    }
}

/*
HINWEIS ZUR KALIBRIERUNG (KG vs Gramm):
Wenn du bei "Gib Masse ein" 1000 (für 1000g) eingibst, wird dein Sensor auf GRAMM kalibriert.
getWeight() liefert dann 1000.0.
getForce() rechnet aber getWeight() * 9.81. -> 9810 Newton! Das ist falsch.

KORREKTUR:
Wenn du getForce() (Newton) nutzen willst, muss getWeight() KILOGRAMM liefern.
Gib bei der Kalibrierung also für 1kg "1.0" ein (oder ändere den Code oben, dass er input/1000 rechnet).
*/
