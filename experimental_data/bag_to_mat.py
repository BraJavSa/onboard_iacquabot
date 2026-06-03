#!/usr/bin/env python3

import os
import numpy as np

from scipy.io import savemat
from scipy.interpolate import interp1d

import rosbag2_py

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


FS = 30.0

PWM_TOPIC = "/pwm_outputs"
ODOM_TOPIC = "/mavros/odometry/in"


def quaternion_to_euler(qx, qy, qz, qw):

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)

    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * np.pi / 2.0
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def interp_signal(t_src, y_src, t_new):

    f = interp1d(
        t_src,
        y_src,
        axis=0,
        bounds_error=False,
        fill_value="extrapolate"
    )

    return f(t_new)


def load_experiment(bag_path):

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    topics = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    if PWM_TOPIC not in topics:
        raise RuntimeError(f"Topic not found: {PWM_TOPIC}")

    if ODOM_TOPIC not in topics:
        raise RuntimeError(f"Topic not found: {ODOM_TOPIC}")

    pwm_type = get_message(topics[PWM_TOPIC])
    odom_type = get_message(topics[ODOM_TOPIC])

    pwm_t = []

    pwm1 = []
    pwm2 = []
    pwm3 = []
    pwm4 = []

    odom_t = []

    x = []
    y = []
    z = []

    qx = []
    qy = []
    qz = []
    qw = []

    roll = []
    pitch = []
    yaw = []

    vx = []
    vy = []
    vz = []

    wx = []
    wy = []
    wz = []

    count = 0

    while reader.has_next():

        topic, data, timestamp = reader.read_next()

        t = timestamp * 1e-9

        if topic == PWM_TOPIC:

            msg = deserialize_message(data, pwm_type)

            if len(msg.data) >= 4:

                pwm_t.append(t)

                pwm1.append(float(msg.data[0]))
                pwm2.append(float(msg.data[1]))
                pwm3.append(float(msg.data[2]))
                pwm4.append(float(msg.data[3]))

        elif topic == ODOM_TOPIC:

            msg = deserialize_message(data, odom_type)

            odom_t.append(t)

            x.append(msg.pose.pose.position.x)
            y.append(msg.pose.pose.position.y)
            z.append(msg.pose.pose.position.z)

            qx_i = msg.pose.pose.orientation.x
            qy_i = msg.pose.pose.orientation.y
            qz_i = msg.pose.pose.orientation.z
            qw_i = msg.pose.pose.orientation.w

            qx.append(qx_i)
            qy.append(qy_i)
            qz.append(qz_i)
            qw.append(qw_i)

            r, p, yy = quaternion_to_euler(
                qx_i,
                qy_i,
                qz_i,
                qw_i
            )

            roll.append(r)
            pitch.append(p)
            yaw.append(yy)

            vx.append(msg.twist.twist.linear.x)
            vy.append(msg.twist.twist.linear.y)
            vz.append(msg.twist.twist.linear.z)

            wx.append(msg.twist.twist.angular.x)
            wy.append(msg.twist.twist.angular.y)
            wz.append(msg.twist.twist.angular.z)

        count += 1

        if count % 100000 == 0:
            print(f"  {count} messages read")

    return {
        "pwm_t": np.asarray(pwm_t),

        "pwm1": np.asarray(pwm1),
        "pwm2": np.asarray(pwm2),
        "pwm3": np.asarray(pwm3),
        "pwm4": np.asarray(pwm4),

        "odom_t": np.asarray(odom_t),

        "x": np.asarray(x),
        "y": np.asarray(y),
        "z": np.asarray(z),

        "qx": np.asarray(qx),
        "qy": np.asarray(qy),
        "qz": np.asarray(qz),
        "qw": np.asarray(qw),

        "roll": np.asarray(roll),
        "pitch": np.asarray(pitch),
        "yaw": np.unwrap(np.asarray(yaw)),

        "vx": np.asarray(vx),
        "vy": np.asarray(vy),
        "vz": np.asarray(vz),

        "wx": np.asarray(wx),
        "wy": np.asarray(wy),
        "wz": np.asarray(wz),
    }


def process_experiment(exp_dir):

    print(f"\nProcessing {exp_dir}")

    D = load_experiment(exp_dir)

    t0 = max(
        D["pwm_t"][0],
        D["odom_t"][0]
    )

    tf = min(
        D["pwm_t"][-1],
        D["odom_t"][-1]
    )

    t_uniform = np.arange(
        t0,
        tf,
        1.0 / FS
    )

    mat = {}

    mat["t"] = t_uniform - t_uniform[0]

    for key in D:

        if key.endswith("_t"):
            continue

        if key.startswith("pwm"):
            source_t = D["pwm_t"]
        else:
            source_t = D["odom_t"]

        mat[key] = interp_signal(
            source_t,
            D[key],
            t_uniform
        )

    output_file = exp_dir + ".mat"

    savemat(
        output_file,
        mat
    )

    print(
        f"Saved {output_file} "
        f"({len(t_uniform)} samples @ {FS:.1f} Hz)"
    )


def main():

    experiments = sorted(
        [
            d
            for d in os.listdir(".")
            if d.startswith("experiment_")
            and os.path.isdir(d)
        ]
    )

    if len(experiments) == 0:
        print("No experiment_* folders found")
        return

    print(f"Found {len(experiments)} experiments")

    for exp in experiments:
        process_experiment(exp)

    print("\nFinished successfully")


if __name__ == "__main__":
    main()