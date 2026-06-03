# bag_to_tlog.py
import struct
import time
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages

output_file = "bote1.tlog"

with open(output_file, "wb") as tlog:
    for schema, channel, message, ros_msg in read_ros2_messages(
        "bote1/bote1_0.mcap",
        topics=["/uas1/mavlink_source"]
    ):
        # timestamp en microsegundos (formato tlog)
        ts = int(message.log_time / 1000)  # nanoseg → microseg
        tlog.write(struct.pack(">q", ts))
        # payload MAVLink raw
        payload = bytes(ros_msg.payload)
        tlog.write(payload)

print(f"Guardado: {output_file}")