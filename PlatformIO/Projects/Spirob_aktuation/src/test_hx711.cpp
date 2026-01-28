#include <Arduino.h>
#include "ForceSensorHX711.h"

// HX711 Pins
#define HX711_DOUT 26  
#define HX711_SCK  27  

#define OFFSET 270331L // Vorher gemessener Offset-Wert (anpassen falls nötig)
#define SCALE  225.64403f // Vorher kalibrierter Scale-Faktor (anpassen nach Kalibrierung)

ForceSensorHX711* sensor;

// Zustände für die Kalibrierung
enum AppState {
    IDLE,               // Normaler Messbetrieb
    CALIB_WAIT_WEIGHT,  // Warte, bis Nutzer Gewicht aufgelegt hat und bestätigt
    CALIB_WAIT_INPUT    // Warte auf Eingabe der Masse via Serial
};

AppState currentState = IDLE;
long calibrationRawValue = 0;  // Speichert den Raw-Wert für die Berechnung

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== HX711 ROBUST TEST + KALIBRIERUNG ===");
    Serial.println("Befehle:");
    Serial.println("  't' -> Tare (Null setzen)");
    Serial.println("  'c' -> Kalibrierung starten");
    
    sensor = new ForceSensorHX711(HX711_DOUT, HX711_SCK, 128,5);

    #if defined(OFFSET) && defined(SCALE)
        sensor->setCalibration(OFFSET, SCALE);
        Serial.print("Vorgegebener Offset gesetzt: ");
        Serial.println(OFFSET);
        Serial.print("Vorgegebener Scale-Faktor gesetzt: ");
        Serial.println(SCALE);
    #endif

    #if defined(OFFSET) && !defined(SCALE)
        sensor->setOffset(OFFSET);
        Serial.print("Vorgegebener Offset gesetzt: ");
        Serial.println(OFFSET);
    #endif

    #if defined(SCALE) && !defined(OFFSET)
        sensor->setScale(SCALE);
        Serial.print("Vorgegebener Scale-Faktor gesetzt: ");
        Serial.println(SCALE);
    #endif

    while(sensor->begin() == false) {
        Serial.println("Warte auf HX711...");
        delay(500);
    }
    Serial.println("✓ HX711 init OK");

    #ifndef OFFSET
        Serial.println("Kein Offset vorgegeben, bitte Tare durchführen ('t')");
        sensor->tare();
        Serial.println("✓ Tare init OK");
    #endif
}

void loop() {
    // ---------------------------------------------------------
    // 1. ZENTRALES SENSOR UPDATE
    // ---------------------------------------------------------
    sensor->update(); // Muss zyklisch laufen, um Daten zu holen

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
                        // Prüfen ob wir valide Daten haben
                            calibrationRawValue = sensor->readRaw();
                            currentState = CALIB_WAIT_INPUT;
                            Serial.printf("\n✓ Raw-Wert gespeichert: %ld\n", calibrationRawValue);
                            Serial.println("4. Gib jetzt die Masse in GRAMM ein (z.B. 1000):");

                    } else {
                        Serial.println("Bitte 's' senden, wenn Gewicht stabil liegt.");
                    }
                    break;

                case CALIB_WAIT_INPUT:
                    float mass = input.toFloat();
                    if (mass > 0) {
                        long offset = sensor->getOffset();
                        float rawDelta = (float)(calibrationRawValue - offset);
                        float newScale = (float)rawDelta / mass;
                        
                        Serial.println("\n🎉 KALIBRIERUNG FERTIG!");
                        Serial.printf("  Masse: %.1f g\n", mass);
                        Serial.printf("  Raw:   %ld\n", calibrationRawValue);
                        Serial.printf("  Scale: %.5f\n", newScale);
                        Serial.println("-------------------------------------");
                        Serial.printf("  Code: sensor->setScale(%.5f);\n", newScale);
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
    // 3. AUSGABE (Rate Limited, nur in IDLE oder WAIT_WEIGHT)
    // ---------------------------------------------------------
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1) { // Max 4Hz Ausgabe
        lastPrint = millis();
        
        if (currentState == IDLE || currentState == CALIB_WAIT_WEIGHT) {
            
                long raw = sensor->readRaw();
                float force = sensor->getForce();
                float weight = sensor->getWeight();
                
                // Schöne Formatierung
                Serial.printf("State:%-12s | Raw:%8ld | Weight:%8.3f g | Force:%8.3f N\n", 
                    (currentState == IDLE) ? "RUN" : "CALIB", 
                    raw, weight, force); // Annahme: Scale ist in Gramm kalibriert -> Anzeige kg
        }
    }
}
