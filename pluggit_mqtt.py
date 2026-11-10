#!/usr/bin/env python3
import json
import minimalmodbus
import serial
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_BASE = "Lueftung/Pluggit"
DISCOVERY_PREFIX = "homeassistant"
POLL_INTERVAL = 60  # Jede Minute abfragen
LOG_FILE_PATH = "/dev/shm/lueftung.log"

# Neues Status-Topic für die Erreichbarkeit
MQTT_AVAILABILITY_TOPIC = f"{MQTT_TOPIC_BASE}/AC200/status"

# --- PLUGGIT / SOLER & PALAU DOMEO 210 FL NAMENZUORDNUNG (DEUTSCH) ---
MODBUS_CONFIG = {
    # 1. DISCRETES INPUT (Digitale Statuswerte - Nur Lesbar - FC 2)
    "DI": {
        1:  {"name": "Geraeteversion Typ", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        2:  {"name": "Freier Kontakt Status", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        5:  {"name": "Frostschutzmodus Aktiv", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        6:  {"name": "Status Abluftventilator", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        7:  {"name": "Status Zuluftventilator", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        9:  {"name": "Status Bypass", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        10: {"name": "Status Sensor Innentemperatur", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        11: {"name": "Status Sensor Aussentemperatur", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        12: {"name": "Status Sensor Ablufttemperatur", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        13: {"name": "Status Sensor Fortlufttemperatur", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
        14: {"name": "Filteralarm Status", "type": "bit", "fc": 2, "ha_type": "binary_sensor"},
    },
    
    # 2. COILS (Schaltbefehle - Les- und Schreibbar - FC 1 / Write FC 5)
    "C": {
        2:  {"name": "Vorheizregister Aktivierung", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        3:  {"name": "Nachheizregister Aktivierung", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        4:  {"name": "Schalter Drehrichtung", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        6:  {"name": "Auswahl Geraeteversion", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        7:  {"name": "Aktivierung Standby Abwesenheit", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        8:  {"name": "Bypass Automatikregelung", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
        9:  {"name": "Bypass Manueller Modus", "type": "bit", "fc": 1, "ha_type": "switch", "write_fc": 5},
    },
    
    # 3. INPUT REGISTER (Analoge Messwerte - Nur Lesbar - FC 4)
    "IR": {
        10: {"name": "Steuerungsart Modus", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor"},
        12: {"name": "Volumenstrom Phasenverschiebung", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "%"},
        13: {"name": "Minimaler Luftvolumenstrom", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "m³/h"},
        14: {"name": "Zeitgesteuerter Intensivlueftung Boost", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "min"},
        15: {"name": "Freie Kuehlung Status", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor"},
        16: {"name": "Aktueller Luftvolumenstrom Gesamt", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "m³/h"},
        17: {"name": "Eingangsregister 17", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor"},
        19: {"name": "Drehzahl Abluftmotor", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "U/min"},
        20: {"name": "Drehzahl Zuluftmotor", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "U/min"},
        21: {"name": "Innentemperatur", "type": "register", "fc": 4, "decimals": 1, "ha_type": "sensor", "unit": "°C"},
        22: {"name": "Aussentemperatur", "type": "register", "fc": 4, "decimals": 1, "ha_type": "sensor", "unit": "°C"},
        23: {"name": "Ablufttemperatur", "type": "register", "fc": 4, "decimals": 1, "ha_type": "sensor", "unit": "°C"},
        24: {"name": "Zulufttemperatur", "type": "register", "fc": 4, "decimals": 1, "ha_type": "sensor", "unit": "°C"},
        25: {"name": "Bypass Klappenstellung", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "%"},
        26: {"name": "Status Vorheizregister", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "%"},
        27: {"name": "Status Nachheizregister", "type": "register", "fc": 4, "decimals": 0, "ha_type": "sensor", "unit": "%"},
    },
    
    # 4. HOLDING REGISTER (Sollwerte/Einstellungen - Les- und Schreibbar - FC 3 / Write FC 6)
    "HR": {
        8:  {"name": "Einstellung Luftvolumenstrom Unsymmetrie", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6, "unit": "%"},
        9:  {"name": "Sollwert Reduzierte Lueftung", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6, "unit": "m³/h"},
        10: {"name": "Sollwert Intensivlueftung Boost 30 Min", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6, "unit": "m³/h"},
        11: {"name": "Sollwert Freie Kuehlung Luftstrom", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6, "unit": "m³/h"},
        15: {"name": "Luefterstufe Vorgabe", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 1, "max": 4, "write_fc": 6}, # Stufenwahl 1-4
        16: {"name": "Minimaler Sollwert 0-10V", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6},
        17: {"name": "Maximaler Sollwert 0-10V", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 100, "write_fc": 6},
        22: {"name": "Bypass Automatik Temperatur Text MIN", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 10, "max": 30, "write_fc": 6, "unit": "°C"},
        23: {"name": "Bypass Automatik Temperatur Tint MIN", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 10, "max": 30, "write_fc": 6, "unit": "°C"},
        24: {"name": "Bypass Manueller Timer Laufzeit", "type": "register", "fc": 3, "decimals": 0, "ha_type": "number", "min": 0, "max": 240, "write_fc": 6, "unit": "min"},
    }
}

# --- ZWISCHENSPEICHER FÜR WERTÄNDERUNGEN ---
last_values = {}

# --- MODBUS SETUP ---
instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
instrument.serial.baudrate = 19200
instrument.serial.bytesize = 8
instrument.serial.parity = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.25
instrument.mode = minimalmodbus.MODE_RTU

# WICHTIG: Wir verwalten das Öffnen und Schließen jetzt komplett manuell per Schleife
instrument.close_port_after_each_call = False

# --- MQTT CALLBACK (SCHREIBEN) ---
def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8")
    topic = msg.topic
    
    for reg_type, addresses in MODBUS_CONFIG.items():
        for addr, cfg in addresses.items():
            clean_name = cfg["name"].replace(" ", "_").replace("?", "")
            expected_cmd_topic = f"{MQTT_TOPIC_BASE}/AC200/{clean_name}/set"
            
            if topic == expected_cmd_topic:
                try:
                    # Vor dem Schreiben Port öffnen
                    instrument.serial.open()
                    
                    if cfg["type"] == "bit":
                        val_to_write = 1 if payload in ["True", "ON", "1"] else 0
                        instrument.write_bit(addr, val_to_write, functioncode=cfg["write_fc"])
                    elif cfg["type"] == "register":
                        val_to_write = int(float(payload))
                        instrument.write_register(addr, val_to_write, functioncode=cfg["write_fc"], signed=True)
                        
                    storage_key = f"{reg_type}_{addr}"
                    last_values[storage_key] = payload
                    
                    client.publish(topic.replace("/set", ""), payload, retain=True)
                except Exception:
                    pass
                finally:
                    # Nach dem Schreibvorgang Port sofort wieder schließen
                    try:
                        instrument.serial.close()
                    except Exception:
                        pass

# --- MQTT SETUP ---
mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_message = on_message

try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
except Exception:
    exit(1)

# --- HOME ASSISTANT DISCOVERY ---
def send_discovery_config():
    device_info = {
        "identifiers": ["pluggit_AC200_system"],
        "name": "Pluggit AC200",
        "model": "AC200 Controller",
        "manufacturer": "Pluggit"
    }

    for reg_type, addresses in MODBUS_CONFIG.items():
        for addr, cfg in addresses.items():
            unique_id = f"pluggit_AC200_{reg_type}_{addr}"
            clean_name = cfg["name"].replace(" ", "_").replace("?", "")
            
            state_topic = f"{MQTT_TOPIC_BASE}/AC200/{clean_name}"
            discovery_topic = f"{DISCOVERY_PREFIX}/{cfg['ha_type']}/{unique_id}/config"
            
            config_payload = {
                "name": cfg["name"],
                "unique_id": unique_id,
                "state_topic": state_topic,
                "device": device_info
            }
            
            if cfg["ha_type"] in ["switch", "number"]:
                cmd_topic = f"{state_topic}/set"
                config_payload["command_topic"] = cmd_topic
                mqtt_client.subscribe(cmd_topic)

            if cfg["type"] == "bit":
                config_payload["payload_on"] = "True"
                config_payload["payload_off"] = "False"
            
            if cfg["ha_type"] == "number":
                config_payload["min"] = cfg.get("min", 0)
                config_payload["max"] = cfg.get("max", 100)
                config_payload["step"] = 1

            if "unit" in cfg:
                config_payload["unit_of_measurement"] = cfg["unit"]

            mqtt_client.publish(discovery_topic, json.dumps(config_payload), retain=True)

send_discovery_config()

# Zu Beginn einmal explizit den Port schließen, da minimalmodbus ihn beim Setup öffnet
try:
    instrument.serial.close()
except Exception:
    pass

time.sleep(4)

# --- DAUERHAFTE ABFRAGE-SCHLEIFE (LESEN) ---
try:
    while True:
        timestamp = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
        log_content = f"--- Durchgang: {timestamp} ---\n"  
        
        try:
            # 1. Port zu Beginn des Durchgangs ÖFFNEN
            instrument.serial.open()
            
            # 2. Alle Werte zügig nacheinander auslesen
            for reg_type, addresses in MODBUS_CONFIG.items():
                for addr, cfg in addresses.items():
                    clean_name = cfg["name"].replace(" ", "_").replace("?", "")
                    topic = f"{MQTT_TOPIC_BASE}/AC200/{clean_name}"
                    storage_key = f"{reg_type}_{addr}"

                    try:
                        if cfg["type"] == "bit":
                            value = instrument.read_bit(addr, cfg["fc"])
                        elif cfg["type"] == "register":
                            value = instrument.read_register(addr, cfg["decimals"], cfg["fc"], signed=True)

                        log_content += f"{reg_type}{addr} {value}\n"

                        str_value = str(value)
                        if storage_key not in last_values or last_values[storage_key] != str_value:
                            mqtt_client.publish(topic, str_value, retain=True)
                            last_values[storage_key] = str_value

                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            # 3. Port am Ende des gesamten Durchgangs SCHLIESSEN
            try:
                instrument.serial.close()
            except Exception:
                pass
        
        # Log-Inhalt in die Datei schreiben
        try:
            with open(LOG_FILE_PATH, "w") as log_file:
                log_file.write(log_content)
        except Exception:
            pass
        
        # 60 Sekunden warten (währenddessen bleibt der serielle Port zu)
        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    pass
finally:
    try:
        instrument.serial.close()
    except Exception:
        pass
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

