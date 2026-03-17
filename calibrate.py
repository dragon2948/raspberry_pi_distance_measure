#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - Calibration Script
===================================================
Place your Bluetooth beacon exactly 1 metre from the Pi and run this.
It collects 100 RSSI samples, removes outliers, and gives you the
TX_POWER value to paste into master.py and slave.py.

Usage:
    sudo python3 calibrate.py
"""

import statistics
from bluepy.btle import Scanner, DefaultDelegate

# -- Configuration ------------------------------------------------------------
TARGET_MAC  = "46:8C:00:00:FE:4D"   # your beacon MAC address
NUM_SAMPLES = 100


# -- Scan Delegate ------------------------------------------------------------

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        pass


# -- Outlier Removal ----------------------------------------------------------

def remove_outliers(readings: list) -> list:
    """
    Remove RSSI readings more than 2 standard deviations from the mean.
    Eliminates sudden spikes for a cleaner TX_POWER estimate.
    """
    avg     = sum(readings) / len(readings)
    std     = statistics.stdev(readings)
    cleaned = [r for r in readings if abs(r - avg) < 1 * std]
    removed = len(readings) - len(cleaned)
    if removed > 0:
        print(f"  Removed {removed} outliers from {len(readings)} readings")
    return cleaned if len(cleaned) > 1 else readings


# -- Main ---------------------------------------------------------------------

def run_calibration():
    print("=" * 50)
    print("  RSE3204 - TX_POWER Calibration")
    print("=" * 50)
    print(f"\nTarget MAC : {TARGET_MAC}")
    print(f"Samples    : {NUM_SAMPLES}")
    print("\nPlace your beacon EXACTLY 1 metre from the Pi.")
    input("Press Enter when ready...")

    scanner  = Scanner().withDelegate(ScanDelegate())
    readings = []

    print(f"\nCollecting {NUM_SAMPLES} samples - keep devices still!")
    while len(readings) < NUM_SAMPLES:
        try:
            devices = scanner.scan(1.0)
        except Exception as e:
            print(f"Scan error: {e}. Retrying...")
            continue

        for dev in devices:
            if dev.addr == TARGET_MAC.lower():
                readings.append(dev.rssi)
                print(f"  Sample {len(readings):>3}/{NUM_SAMPLES}  RSSI = {dev.rssi} dBm", end="\r")
                break

    print("\n")

    # Stats before cleaning
    simple_avg = sum(readings) / len(readings)
    print(f"Raw simple average     = {simple_avg:.2f} dBm")

    # Remove outliers
    clean_readings = remove_outliers(readings)
    clean_avg      = sum(clean_readings) / len(clean_readings)
    print(f"Cleaned simple average = {clean_avg:.2f} dBm")
    print(f"Samples kept           = {len(clean_readings)}/{NUM_SAMPLES}")

    print(f"\n{'=' * 50}")
    print(f"  Recommended TX_POWER = {round(clean_avg)}")
    print(f"{'=' * 50}")
    print(f"\nSet this in both master.py and slave.py:")
    print(f"  TX_POWER = {round(clean_avg)}")


if __name__ == "__main__":
    try:
        run_calibration()
    except KeyboardInterrupt:
        print("\nCalibration stopped.")
    except Exception as e:
        print(f"\nError: {e}")
