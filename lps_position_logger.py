import logging
import sys
import time
from threading import Event
import csv
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

# ---------------- Configuration ----------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
deck_attached_event = Event()

logging.basicConfig(level=logging.INFO)

position_estimate = [0.0, 0.0, 0.0]
vel_log = []  # Store velocity samples
# ------------------------------------------------


def log_pos_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']


def log_vel_callback(timestamp, data, logconf):
    vel_log.append([
        timestamp,
        data['stateEstimate.vx'],
        data['stateEstimate.vy'],
        data['stateEstimate.vz']
    ])
    print(f"[{timestamp}] vx={data['stateEstimate.vx']:.3f} m/s, "
          f"vy={data['stateEstimate.vy']:.3f} m/s, "
          f"vz={data['stateEstimate.vz']:.3f} m/s")


def param_deck_loco(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print('✅ Loco Positioning deck detected.')
    else:
        print('❌ No Loco deck attached.')


# ------------------------------------------------
#                   MAIN LOGIC
# ------------------------------------------------
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print(f'Connecting to Crazyflie at URI {URI}...')
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # --- Wait for Loco deck ---
        cf.param.add_update_callback(
            group='deck',
            name='bcLoco',
            cb=param_deck_loco
        )
        print("Waiting for Loco deck...")
        if not deck_attached_event.wait(timeout=5):
            print('No Loco deck detected. Exiting.')
            sys.exit(1)

        # --- Configure estimator ---
        print("Configuring Kalman estimator for LPS...")
        cf.param.set_value('stabilizer.estimator', '2')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(1.0)

        # --- Set PID values ---
        print("Setting PID parameters...")

        # Attitude PID (Roll and Pitch)
        cf.param.set_value('pid_attitude.roll_kp', 8.0)
        cf.param.set_value('pid_attitude.roll_kd', 0.5)
        cf.param.set_value('pid_attitude.pitch_kp', 8.0)
        cf.param.set_value('pid_attitude.pitch_kd', 0.5)

        # Velocity PID (body X/Y)
        cf.param.set_value('velCtlPid.vxKp', 35.0)
        cf.param.set_value('velCtlPid.vxKd', 0.5)
        cf.param.set_value('velCtlPid.vyKp', 35.0)
        cf.param.set_value('velCtlPid.vyKd', 0.5)

        time.sleep(0.5)

        # --- Verify PID values ---
        def get_param_value(name):
            return float(cf.param.get_value(name))

        pid_values = {
            "pid_attitude.roll_kp": get_param_value("pid_attitude.roll_kp"),
            "pid_attitude.roll_kd": get_param_value("pid_attitude.roll_kd"),
            "pid_attitude.pitch_kp": get_param_value("pid_attitude.pitch_kp"),
            "pid_attitude.pitch_kd": get_param_value("pid_attitude.pitch_kd"),
            "velCtlPid.vxKp": get_param_value("velCtlPid.vxKp"),
            "velCtlPid.vxKd": get_param_value("velCtlPid.vxKd"),
            "velCtlPid.vyKp": get_param_value("velCtlPid.vyKp"),
            "velCtlPid.vyKd": get_param_value("velCtlPid.vyKd"),
        }

        print("\n🧩 PID Parameters Set:")
        for name, val in pid_values.items():
            print(f"  {name:25s} = {val}")

        # Basic sanity checks
        assert abs(pid_values["velCtlPid.vxKp"] - 35.0) < 0.1
        assert abs(pid_values["velCtlPid.vyKp"] - 35.0) < 0.1
        assert abs(pid_values["velCtlPid.vxKd"] - 0.5) < 0.1
        assert abs(pid_values["velCtlPid.vyKd"] - 0.5) < 0.1

        print("\n✅ PID parameters verified successfully.\n")

        # --- Setup velocity logging ---
        logconf = LogConfig(name='Velocity', period_in_ms=100)
        logconf.add_variable('stateEstimate.vx', 'float')
        logconf.add_variable('stateEstimate.vy', 'float')
        logconf.add_variable('stateEstimate.vz', 'float')
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_vel_callback)
        logconf.start()

        # --- Setup position logging for reference ---
        pos_log = LogConfig(name='Position', period_in_ms=100)
        pos_log.add_variable('stateEstimate.x', 'float')
        pos_log.add_variable('stateEstimate.y', 'float')
        pos_log.add_variable('stateEstimate.z', 'float')
        cf.log.add_config(pos_log)
        pos_log.data_received_cb.add_callback(log_pos_callback)
        pos_log.start()

        print("Getting initial position estimate...")
        time.sleep(2.0)
        start_x, start_y, start_z = position_estimate
        print(f"Current position: x={start_x:.2f}, y={start_y:.2f}, z={start_z:.2f}")

        target_z = start_z + 0.5
        print(f"Target position:  z={target_z:.2f}")

        # --- Movement sequence ---
        with PositionHlCommander(scf, default_height=start_z) as pc:
            print("Ascending 50 cm...")
            pc.take_off(absolute_height_m=target_z, duration_s=1)

            time.sleep(3.0)
            print("Hovering for 3 seconds...")
            time.sleep(3.0)

            print("Landing...")
            pc.land()
            time.sleep(2.0)

        # --- Stop logging ---
        logconf.stop()
        pos_log.stop()

        print("✅ Flight complete. Saving velocity log...")

        # --- Save velocity data ---
        with open('velocity_log.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'vx', 'vy', 'vz'])
            writer.writerows(vel_log)

        # --- Plot velocity data ---
        timestamps = [v[0] for v in vel_log]
        vx = [v[1] for v in vel_log]
        vy = [v[2] for v in vel_log]
        vz = [v[3] for v in vel_log]

        plt.figure(figsize=(8, 5))
        plt.plot(timestamps, vx, label='vx')
        plt.plot(timestamps, vy, label='vy')
        plt.plot(timestamps, vz, label='vz')
        plt.xlabel('Time (ms)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Crazyflie Velocity vs Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        print("📈 Velocity graph displayed and data saved to velocity_log.csv")

