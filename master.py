#!/usr/bin/env python3
"""
RSE3204 Wireless Localisation - MASTER (Pi A)
=============================================
Run this on Pi A.
It:
  1. Scans for the target Bluetooth device and measures dXA via RSSI
     (using a Kalman Filter to reduce noise)
  2. Asks the operator to enter dAB (Pi A → Pi B baseline distance)
  3. Sends a GET_DISTANCE request to Slave (Pi B) over UART
  4. Receives dXB from Pi B over UART
  5. Computes θAB using the Law of Cosines
  6. Prints a formatted summary table

Usage:
    sudo python3 master.py

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
UART_TIMEOUT  = None      # no timeout - wait as long as slave needs (100 samples takes a while)

# -- Bluetooth Configuration --------------------------------------------------
TARGET_MAC   = "46:8C:00:00:FE:4D"  # your beacon MAC address
TX_POWER     = -63                   # RSSI at 1 metre (calibrate for your device)
PATH_LOSS    = 2.0                  # Path loss exponent (2.0 = free space)
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
    estimate          = readings[0]   # start with first reading
    error             = 1.0
    process_noise     = 0.01          # assume RSSI is fairly stable
    measurement_noise = 2.0           # RSSI is noisy, moderately high

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
    cleaned = [r for r in readings if abs(r - avg) < 1 * std]
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


def measure_distance_bluetooth(label: str, samples: int = NUM_SAMPLES) -> float:
    """
    Scan for TARGET_MAC, collect RSSI readings, apply Kalman Filter,
    and return estimated distance in metres.
    """
    print(f"[Master] Scanning for Bluetooth device ({TARGET_MAC}) to measure {label}...")
    print(f"[Master] Collecting {samples} samples - keep devices still!")

    scanner = Scanner().withDelegate(ScanDelegate())
    readings = []

    while len(readings) < samples:
        try:
            devices = scanner.scan(1.0)
        except Exception as e:
            print(f"[Master] Scan error: {e}. Retrying...")
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

    print(f"[Master] Simple average RSSI  = {simple_avg:.2f} dBm")
    print(f"[Master] Kalman filtered RSSI = {filtered_rssi:.2f} dBm")
    print(f"[Master] {label} = {distance:.3f} m")
    return distance


# -- Manual distance input ----------------------------------------------------

def prompt_distance(label: str) -> float:
    """Prompt operator for a distance value (used for dAB baseline)."""
    while True:
        try:
            val = float(input(f"[Master] Enter {label} (metres): "))
            if val < 0:
                print("  Distance cannot be negative. Try again.")
                continue
            return val
        except ValueError:
            print("  Invalid input. Please enter a number.")


# -- UART communication -------------------------------------------------------

def fetch_dxb_from_slave(uart: serial.Serial) -> float:
    """Send GET_DISTANCE request over UART and wait for dXB response."""
    request = json.dumps({"cmd": "GET_DISTANCE"}) + "\n"
    print("[Master] Sending GET_DISTANCE request to slave over UART ...")
    uart.write(request.encode())
    uart.flush()

    print("[Master] Waiting for slave response (this may take a while - slave is scanning 100 samples)...")
    raw = uart.readline()

    response = json.loads(raw.decode().strip())

    if "error" in response:
        raise RuntimeError(f"Slave returned error: {response['error']}")

    dxb = float(response["dxb"])
    print(f"[Master] Received dXB = {dxb:.3f} m from slave.")
    return dxb


# -- Geometry -----------------------------------------------------------------

def compute_angle(dxa: float, dxb: float, dab: float):
    """
    Compute theta_AB (angle at vertex X) using the Law of Cosines:

        cos(theta) = (dXA^2 + dXB^2 - dAB^2) / (2 * dXA * dXB)

    Returns angle in degrees, or None if degenerate.
    """
    if dxa == 0 or dxb == 0:
        return None

    cos_theta = (dxa**2 + dxb**2 - dab**2) / (2 * dxa * dxb)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def validate_triangle(dxa: float, dxb: float, dab: float) -> bool:
    """Triangle inequality check."""
    return (dxa + dxb > dab) and (dxa + dab > dxb) and (dxb + dab > dxa)


# -- Display ------------------------------------------------------------------

def print_results(dxa: float, dxb: float, dab: float, theta):
    sep = "=" * 46
    print(f"\n  +{sep}+")
    print(f"  |{'RSE3204 - Localisation Results':^46}|")
    print(f"  +{sep}+")
    print(f"  |  {'dXA  (Pi A -> Bluetooth device)':<38} {dxa:>5.3f} m  |")
    print(f"  |  {'dXB  (Pi B -> Bluetooth device)':<38} {dxb:>5.3f} m  |")
    print(f"  |  {'dAB  (Pi A -> Pi B baseline)':<38} {dab:>5.3f} m  |")
    print(f"  +{sep}+")
    if theta is not None:
        print(f"  |  {'Angle at device X (theta_AB)':<38} {theta:>5.2f} deg  |")
    else:
        print(f"  |  {'Angle at device X (theta_AB)':<38} {'N/A':>7}  |")
    print(f"  +{sep}+")

    print("\n  Triangle layout (not to scale):\n")
    print("        Pi A")
    print("         |\\")
    print(f"   dXA={dxa:.2f}m |  \\ dAB={dab:.2f}m")
    print("         |    \\")
    print("    device X----Pi B")
    print(f"        dXB={dxb:.2f}m")
    if theta is not None:
        print(f"\n  Angle at device X = {theta:.2f} degrees")
    print()


# -- Main ---------------------------------------------------------------------

def run_master():
    print("=" * 50)
    print("  RSE3204 Wireless Localisation - MASTER (Pi A)")
    print("=" * 50)

    # Step 1: Measure dXA via Bluetooth RSSI with Kalman Filter
    dxa = measure_distance_bluetooth("dXA")

    # Step 2: Get baseline dAB from operator (tape measure)
    dab = prompt_distance("dAB  [Pi A -> Pi B baseline]")

    # Step 3: Open UART and fetch dXB from slave
    print(f"\n[Master] Opening UART on {UART_PORT} at {UART_BAUDRATE} baud ...")
    with serial.Serial(UART_PORT, UART_BAUDRATE, timeout=UART_TIMEOUT) as uart:
        dxb = fetch_dxb_from_slave(uart)

    # Step 4: Validate triangle
    if not validate_triangle(dxa, dxb, dab):
        print("\n  WARNING: Distances do not form a valid triangle.")
        print("     Angle cannot be computed. Check your measurements.\n")
        theta = None
    else:
        theta = compute_angle(dxa, dxb, dab)

    # Step 5: Display results
    print_results(dxa, dxb, dab, theta)


if __name__ == "__main__":
    try:
        run_master()
    except KeyboardInterrupt:
        print("\n[Master] Stopped.")
    except serial.SerialException as e:
        print(f"\n[Master] UART error: {e}")
        print("  Make sure UART is enabled (raspi-config) and you are running with sudo.")
    except TimeoutError as e:
        print(f"\n[Master] Timeout: {e}")
    except Exception as e:
        print(f"\n[Master] Error: {e}")