import time
import logging
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# ---------------- Configuration ----------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
THRUST_PERCENT = 0.63       # one is 70% thrust for hover, other is about 63
THRUST_VALUE = int(65535 * THRUST_PERCENT)
HOLD_TIME = 5.0             # seconds
# ------------------------------------------------

logging.basicConfig(level=logging.INFO)

def main():
    print(f"[INFO] Initializing CRTP drivers...")
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print(f"[INFO] Connecting to Crazyflie at {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        print("[INFO] Connected successfully.")

        # Allow full connection setup
        print("[DEBUG] Waiting 2 seconds for full connection setup...")
        time.sleep(2.0)

        # Warm up with zero-thrust packets
        print("[DEBUG] Sending zero-thrust warm-up packets...")
        for i in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

        print(f"[INFO] Applying thrust = {THRUST_VALUE} ({THRUST_PERCENT*100:.1f}%) for {HOLD_TIME} seconds...")

        start_time = time.time()
        try:
            while time.time() - start_time < HOLD_TIME:
                cf.commander.send_setpoint(0, 0, 0, THRUST_VALUE)
                time.sleep(0.02)  # send continuously (~50 Hz)
        except KeyboardInterrupt:
            print("\n[WARN] Interrupted by user (Ctrl+C). Cutting thrust immediately...")

        # Always stop motors before exit
        print("[INFO] Stopping motors...")
        for _ in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)
        cf.commander.send_stop_setpoint()

        print("[SUCCESS] Test complete. Disconnecting...")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[WARN] Force-exiting safely...")
        sys.exit(0)
