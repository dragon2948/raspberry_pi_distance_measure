#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - SLAVE (Pi B)
=============================================
Run this on Pi B.
Waits for a request from master over UART, then scans for
the Bluetooth device via RSSI (using Kalman Filter), and sends dXB back.

Usage:
    sudo python3 slave.py

Requirements:
    pip install pyserial
    sudo apt-get install python3-pip libglib2.0-dev
    (bluepy installed from source - see project README)
"""

import serial
import json
import math
from bluepy.btle import Scanner, DefaultDelegate

# -- UART Configuration -------------------------------------------------------
UART_PORT     = "/dev/serial0"
UART_BAUDRATE = 9600
UART_TIMEOUT  = 30        # seconds to wait for a message

# -- Bluetooth Configuration --------------------------------------------------
TARGET_MAC   = "46:8C:00:00:FE:4D"  # your beacon MAC address
TX_POWER     = -70                   # RSSI at 1 metre (calibrate for your device)
PATH_LOSS    = 2.0                   # Path loss exponent (2.0 = free space)
NUM_SAMPLES  = 100                   # Number of RSSI readings to collect


# -- Kalman Filter ------------------------------------------------------------

def kalman_filter(readings: list) -> float:
    """
    Apply a 1D Kalman Filter to a list of RSSI readings.

    Key variables:
      estimate          : current best guess of the true RSSI
      error             : how uncertain we are about our estimate
      process_noise     : how much the true RSSI might drift between readings
                          (small = assume RSSI is stable)
      measurement_noise : how noisy/unreliable the sensor is
                          (larger = trust new readings less)

    Each iteration:
      1. Kalman Gain = error / (error + measurement_noise)
         how much to trust the new reading vs current estimate
      2. estimate = estimate + gain * (new_reading - estimate)
         blend estimate toward new reading weighted by gain
      3. error = (1 - gain) * error
         shrinks as we become more confident
    """
    estimate          = readings[0]
    error             = 1.0
    process_noise     = 0.01
    measurement_noise = 2.0

    for rssi in readings[1:]:
        # Prediction step: uncertainty grows slightly each step
        error = error + process_noise

        # Update step
        gain     = error / (error + measurement_noise)
        estimate = estimate + gain * (rssi - estimate)
        error    = (1 - gain) * error

    return estimate


# -- Outlier Removal ----------------------------------------------------------

def remove_outliers(readings: list) -> list:
    """
    Remove RSSI readings that are more than 2 standard deviations
    from the mean. This eliminates sudden spikes before Kalman Filter.
    """
    import statistics
    avg     = sum(readings) / len(readings)
    std     = statistics.stdev(readings)
    cleaned = [r for r in readings if abs(r - avg) < 2 * std]
    removed = len(readings) - len(cleaned)
    if removed > 0:
        print(f"  Removed {removed} outliers from {len(readings)} readings")
    return cleaned if len(cleaned) > 1 else readings   # fallback if too many removed


# -- Bluetooth RSSI to Distance -----------------------------------------------

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


def rssi_to_distance(rssi: float) -> float:
    """
    Convert RSSI (dBm) to estimated distance (metres) using the log-distance
    path loss model:

        distance = 10 ^ ((TX_POWER - RSSI) / (10 * PATH_LOSS))

    TX_POWER  : RSSI measured at exactly 1 metre (calibrate this!)
    PATH_LOSS : environment constant - 2.0 free space, 2.7-3.5 indoors
    """
    return 10 ** ((TX_POWER - rssi) / (10 * PATH_LOSS))


def measure_distance_bluetooth(samples: int = NUM_SAMPLES) -> float:
    """
    Scan for TARGET_MAC, collect RSSI readings, apply Kalman Filter,
    and return estimated distance in metres.
    """
    print(f"[Slave] Scanning for Bluetooth device ({TARGET_MAC}) to measure dXB...")
    print(f"[Slave] Collecting {samples} samples - keep devices still!")

    scanner = Scanner().withDelegate(ScanDelegate())
    readings = []

    while len(readings) < samples:
        try:
            devices = scanner.scan(1.0)
        except Exception as e:
            print(f"[Slave] Scan error: {e}. Retrying...")
            continue

        for dev in devices:
            if dev.addr == TARGET_MAC.lower():
                readings.append(dev.rssi)
                print(f"  Sample {len(readings):>3}/{samples}  RSSI = {dev.rssi} dBm", end="\r")
                break

    print()

    # Step 1: Remove outliers
    clean_readings = remove_outliers(readings)

    # Step 2: Apply Kalman Filter on clean readings
    filtered_rssi = kalman_filter(clean_readings)
    simple_avg    = sum(readings) / len(readings)
    distance      = rssi_to_distance(filtered_rssi)

    print(f"[Slave] Simple average RSSI  = {simple_avg:.2f} dBm")
    print(f"[Slave] Kalman filtered RSSI = {filtered_rssi:.2f} dBm")
    print(f"[Slave] dXB = {distance:.3f} m")
    return distance


# -- Main slave loop ----------------------------------------------------------

def run_slave():
    print("=" * 50)
    print("  RSE3204 Wireless Localisation - SLAVE (Pi B)")
    print("=" * 50)
    print(f"[Slave] Opening UART on {UART_PORT} at {UART_BAUDRATE} baud ...")

    with serial.Serial(UART_PORT, UART_BAUDRATE, timeout=UART_TIMEOUT) as uart:
        import time
        time.sleep(2)
        uart.reset_input_buffer()
        print("[Slave] UART ready. Waiting for master request ...\n")

        while True:
            # Wait for a line from master
            raw = uart.readline()
            if not raw:
                print("[Slave] Timeout waiting for master. Still waiting ...")
                continue

            decoded = raw.decode().strip()
            if not decoded:
                continue

            try:
                request = json.loads(decoded)
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"[Slave] Bad data received: {e}. Ignoring.")
                continue

            print(f"[Slave] Request received: {request}")

            if request.get("cmd") == "GET_DISTANCE":
                # Measure dXB via Bluetooth RSSI with Kalman Filter
                try:
                    dxb = measure_distance_bluetooth()
                except Exception as e:
                    error = json.dumps({"error": f"Bluetooth scan failed: {e}"}) + "\n"
                    uart.write(error.encode())
                    uart.flush()
                    print(f"[Slave] Bluetooth error: {e}")
                    continue

                # Send response back to master
                response = json.dumps({"dxb": dxb}) + "\n"
                uart.write(response.encode())
                uart.flush()
                print(f"[Slave] Sent dXB = {dxb:.3f} m to master.\n")
                print("[Slave] Waiting for next request ...\n")

            else:
                error = json.dumps({"error": "unknown command"}) + "\n"
                uart.write(error.encode())
                uart.flush()
                print("[Slave] Unknown command. Sent error to master.")


# -- Entry point --------------------------------------------------------------

if __name__ == "__main__":
    try:
        run_slave()
    except KeyboardInterrupt:
        print("\n[Slave] Stopped.")
    except serial.SerialException as e:
        print(f"\n[Slave] UART error: {e}")
        print("  Make sure UART is enabled (raspi-config) and you are running with sudo.")
