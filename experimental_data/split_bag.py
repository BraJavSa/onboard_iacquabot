#!/usr/bin/env python3

import os
import sys
import shutil

import rosbag2_py


def main():

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 split_bag.py <bag_folder> [num_experiments]")
        sys.exit(1)

    bag_path = sys.argv[1]
    num_experiments = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    if not os.path.exists(bag_path):
        print(f"Bag not found: {bag_path}")
        sys.exit(1)

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="mcap"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    print("Reading timestamps...")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()

    first_time = None
    last_time = None

    while reader.has_next():
        _, _, timestamp = reader.read_next()

        if first_time is None:
            first_time = timestamp

        last_time = timestamp

    duration_ns = last_time - first_time
    duration_s = duration_ns / 1e9

    print(f"Duration: {duration_s:.3f} s")

    segment_ns = duration_ns / num_experiments

    print(
        f"Creating {num_experiments} experiments "
        f"({segment_ns / 1e9:.3f} s each)"
    )

    writers = []

    for i in range(num_experiments):

        out_dir = f"experiment_{i+1:02d}"

        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)

        writer = rosbag2_py.SequentialWriter()

        writer.open(
            rosbag2_py.StorageOptions(
                uri=out_dir,
                storage_id="mcap"
            ),
            converter_options
        )

        for topic in topics:
            writer.create_topic(topic)

        writers.append(writer)

    print("Re-reading bag and splitting...")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_count = 0

    while reader.has_next():

        topic_name, serialized_data, timestamp = reader.read_next()

        idx = int((timestamp - first_time) / segment_ns)

        if idx >= num_experiments:
            idx = num_experiments - 1

        writers[idx].write(
            topic_name,
            serialized_data,
            timestamp
        )

        msg_count += 1

        if msg_count % 50000 == 0:
            print(f"{msg_count} messages processed")

    print()
    print("Finished successfully")
    print(f"Original bag preserved: {bag_path}")

    for i in range(num_experiments):
        print(f"experiment_{i+1:02d}")


if __name__ == "__main__":
    main()