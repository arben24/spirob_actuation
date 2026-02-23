#!/usr/bin/env python3
"""
SpiRob Live Monitor - Einfaches Beispiel für ESP32 Kommunikation (2 Motoren)
Zeigt Telemetrie-Daten beider Motoren in Echtzeit und erlaubt interaktive Kommandos.

Verwendung:
    python3 live_monitor_simple.py
    
Kommandos während der Ausführung:
    f <m|all> <N>      - Kraft-Sollwert setzen (z.B. 'f 0 5.0', 'f all 3')
    start <m|all>      - Kraftregelung starten (PID)
    pid <m> <kp> <ki> <kd> - PID-Regler einstellen (z.B. 'pid 0 30 0.5 0')
    v <m|all> <speed>  - Direkter Speed (z.B. 'v 0 800', 'v all -500')
    fl <N>             - Kraft-Limit für Step-Response (z.B. 'fl 15')
    c <m|all>          - Homing
    r <m|all>          - Return to rope length 0
    stop [m]           - Motor(en) stoppen + Kraftregelung aus
    q                  - Beenden
"""

import serial
import struct
import sys
import select
import time

# ============================================================================
# KONFIGURATION
# ============================================================================

PORT = '/dev/ttyUSB0'       # Linux: /dev/ttyUSB0, Windows: COM3
BAUDRATE = 460800
HEADER = b'\xaa\x55'        # Binär-Header für Status-Pakete
STEP_END = b'\xbb\x66'      # Step-End-Header
STRUCT_FORMAT = '<I ff ff'  # uint32, 2×float32(force), 2×float32(rope)
STRUCT_SIZE = 20            # 4 + 4*2 + 4*2 = 20 Bytes

# ============================================================================
# HILFSFUNKTIONEN
# ============================================================================

def sync_to_header(ser_port, timeout=0.1):
    """
    Synchronisiert auf Header-Pattern (0xAA 0x55 oder 0xBB 0x66).
    
    Returns:
        bytes|None: Gefundener Header oder None bei Timeout
    """
    sync_buf = b''
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        byte = ser_port.read(1)
        if not byte:
            continue
        
        sync_buf += byte
        if len(sync_buf) > 2:
            sync_buf = sync_buf[-2:]
        
        if sync_buf == HEADER:
            return HEADER
        if sync_buf == STEP_END:
            return STEP_END
    
    return None

# ============================================================================
# HAUPTPROGRAMM
# ============================================================================

def main():
    """Hauptschleife: Telemetrie empfangen + interaktive Kommandos."""
    
    # Serielle Verbindung öffnen
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.001)
        time.sleep(0.1)  # MCU Stabilisierung
        ser.reset_input_buffer()
        print(f"✅ Verbunden mit {PORT} @ {BAUDRATE} baud (2 Motoren)")
        print("\n📋 Verfügbare Kommandos:")
        print("   -- Kraftregelung --")
        print("   f <m|all> <N>           - Kraft-Sollwert (z.B. 'f 0 5.0', 'f all 3')")
        print("   start <m|all>           - Kraftregelung starten (PID)")
        print("   pid <m> <kp> <ki> <kd>  - PID tunen (z.B. 'pid 0 30 0.5 0')")
        print("   -- Direkte Steuerung --")
        print("   v <m|all> <speed>       - Direkter Speed (deaktiviert Kraftregelung)")
        print("   fl <N>                  - Kraft-Limit für Step-Response (z.B. 'fl 15')")
        print("   -- Referenzierung --")
        print("   c <m|all>               - Homing / Referenzfahrt")
        print("   r <m|all>               - Return to rope length 0")
        print("   stop [m]                - Motor(en) stoppen + Kraftregelung aus")
        print("   q                       - Beenden\n")
    except serial.SerialException as e:
        print(f"❌ Fehler beim Öffnen von {PORT}: {e}")
        return
    
    try:
        print("📡 Empfange Daten... (Strg+C zum Beenden)\n")
        
        while True:
            # ========================================================
            # 1. TELEMETRIE EMPFANGEN (non-blocking)
            # ========================================================
            
            if ser.in_waiting >= 22:  # Mindestens Header (2) + Data (20) Bytes
                # Header synchronisieren
                hdr = sync_to_header(ser, timeout=0.001)
                if hdr is None:
                    continue
                
                if hdr == HEADER:
                    # Nutzdaten lesen (20 Bytes)
                    data = ser.read(STRUCT_SIZE)
                    if len(data) < STRUCT_SIZE:
                        continue
                    
                    try:
                        ts_us, f0, f1, r0, r1 = struct.unpack(STRUCT_FORMAT, data)
                        print(f"\r📡 {ts_us:10d}us | M0: {f0:6.2f}N {r0:7.1f}mm | M1: {f1:6.2f}N {r1:7.1f}mm  ", 
                              end='', flush=True)
                    except struct.error as e:
                        print(f"\n⚠️ Dekodierungs-Fehler: {e}")
                        ser.reset_input_buffer()
                
                elif hdr == STEP_END:
                    motor_byte = ser.read(1)
                    motor_idx = motor_byte[0] if motor_byte else -1
                    print(f"\n🏁 Step-Ende: Motor {motor_idx}")
            
            # ========================================================
            # 2. USER-EINGABE VERARBEITEN (non-blocking)
            # ========================================================
            
            # Prüfe ob Tastatur-Eingabe verfügbar (ohne zu blockieren)
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            
            if ready:
                user_input = sys.stdin.readline().strip()
                
                # Quit-Befehl
                if user_input.lower() == 'q':
                    print("\n👋 Beende...")
                    break
                
                # Kommando an ESP32 senden
                if user_input:
                    command = user_input if user_input.endswith('\n') else user_input + '\n'
                    ser.write(command.encode('ascii'))
                    print(f"\n→ Gesendet: {user_input}")
            
            # Kleine Pause um CPU zu schonen
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\n\n👋 Programm durch Nutzer beendet (Strg+C)")
    
    except Exception as e:
        print(f"\n❌ Unerwarteter Fehler: {e}")
    
    finally:
        # Serielle Verbindung schließen
        if ser.is_open:
            ser.close()
            print("✅ Serielle Verbindung geschlossen")

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()
