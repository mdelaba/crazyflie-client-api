import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.INFO)

position_estimate = [0.0, 0.0, 0.0]


def log_pos_callback(timestamp, data, logconf):
    """Updates global position estimate."""
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']


def param_deck_loco(_, value_str):
    """Detects whether the Loco Positioning deck is attached."""
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print('✅ Loco Positioning deck detected.')
    else:
        print('❌ No Loco deck attached.')


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print(f'Connecting to Crazyflie at URI {URI}...')
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Wait for Loco deck
        scf.cf.param.add_update_callback(
            group='deck',
            name='bcLoco',
            cb=param_deck_loco
        )

        print("Waiting for Loco deck...")
        if not deck_attached_event.wait(timeout=5):
            print('No Loco deck detected. Exiting.')
            sys.exit(1)

        # --- Setup estimator ---
        print("Configuring Kalman estimator for LPS...")
        scf.cf.param.set_value('stabilizer.estimator', '2')   # Kalman filter
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(1.0)

        # --- Setup logging ---
        logconf = LogConfig(name='Position', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.start()

        print("Getting initial position estimate...")
        time.sleep(2.0)
        start_x, start_y, start_z = position_estimate
        print(f"Current position: x={start_x:.2f}, y={start_y:.2f}, z={start_z:.2f}")

        target_z = start_z + 0.5  # Move 50 cm up
        print(f"Target position:  z={target_z:.2f}")

        # --- Command the movement ---
        with PositionHlCommander(scf, default_height=start_z) as pc:
            pc.takeoff(self, absolute_height_m=target_z, duration_s=1)
            print("Ascending 50 cm...")
            time.sleep(3.0)

            print("Hovering for 3 seconds...")
            time.sleep(3.0)

            print("Landing...")
            pc.land()
            time.sleep(2.0)

        logconf.stop()
        print("✅ Flight complete.")

