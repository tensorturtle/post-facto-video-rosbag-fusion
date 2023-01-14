# fuse.py 
# Read an mcap rosbag and mp4 files, then combine them into a new mcap rosbag
#
# Run this script from within a Docker container which has ROS2 Galactic and mcap plugin installed. See Dockerfile.galactic for the Dockerfile.
import argparse
import time
import sys
import glob 

from rclpy.serialization import serialize_message, deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


def form_compressed_image_msg(frame, encode_param):
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data.frombytes(cv2.imencode('.jpg', frame, encode_param)[1])
    return msg

def form_image_msg(frame):
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    flattened = np.ravel(frame)
    msg.data.frombytes(flattened)
    return msg

def parse_start_time_from_filename(filename):
    try:
        # filename format:path/to/file/csi-1673684144607-ms-001-minutes.mp4
        split_filename = filename.split('/')[-1].split('-')
        camera_type = split_filename[0]
        start_time_ms = int(split_filename[1])
        minutes_elapsed = int(split_filename[3])
    except ValueError:
        print("ERROR: Unable to parse start time from filename. Please check video filename format example: csi-1673684144607-ms-001-minutes.mp4")
        sys.exit(0)
    return start_time_ms


def main(args):
    # time stamp is in nanoseconds
    frame_time_nanos = 1_000_000_000 // int(args.target_fps)
    if not args.raw:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]

    
    # get mp4 files within args.mp4 directory
    mp4_files = glob.glob(args.mp4 + "/*.mp4")
    mp4_files.sort()

    start_time_nanos = int(parse_start_time_from_filename(mp4_files[0]) * 1e6)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    # create a topic
    message_type = "sensor_msgs/msg/CompressedImage" if not args.raw else "sensor_msgs/msg/Image"
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=args.topic_name,
            type=message_type,
            serialization_format="cdr",
        )
    )

    elapsed_frames = 0
    written_frames = 0

    for i, video_file in enumerate(mp4_files):
        elapsed_frames = 0
        print(f"Processing {i+1}/{len(mp4_files)} videos          ")
        cap = cv2.VideoCapture(video_file)

        num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        fps = int(cap.get(cv2.CAP_PROP_FPS))
        if args.target_fps > fps:
            print("ERROR: Target FPS is higher than input FPS.")
            sys.exit(0)
        if fps / args.target_fps != fps // args.target_fps:
            print("ERROR: Input FPS is not divisible by target FPS.")
            sys.exit(0)

        skip_n_frames = 0 if fps == args.target_fps else int(fps / args.target_fps)-1

        while(cap.isOpened()):
            print(f"Processing {elapsed_frames}/{num_frames} frames", end='\r')

            if skip_n_frames > 0:
                for _ in range(skip_n_frames):
                    ret, frame = cap.read()
                    elapsed_frames += 1
                    if not ret:
                        break

            ret, frame = cap.read()

            if ret:
                # skip frames
                if args.target_resolution != 'None':
                    if frame.shape[0] < args.target_resolution[0] or frame.shape[1] < args.target_resolution[1]:
                        print("WARNING: Target resolution is larger than frame resolution. Please check --target-resolution argument.")
                    
                frame = cv2.resize(frame, (args.target_resolution[1], args.target_resolution[0]), interpolation=cv2.INTER_AREA)

                if args.raw:
                    msg = form_image_msg(frame)
                else:
                    msg = form_compressed_image_msg(frame, encode_param)

                timestamp = start_time_nanos + written_frames* frame_time_nanos + int(args.video_timestamp_offset * 1e6)

                writer.write(args.topic_name, serialize_message(msg), timestamp)

                elapsed_frames += 1
                written_frames += 1
            else:
                break
                
    # read a different bag and write to current merged bag
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topics = []

    topic_types = reader.get_all_topics_and_types()

    while reader.has_next():
        topic, msg, timestamp = reader.read_next()
        if topic not in topics:
            topics.append(topic)
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic,
                    type=typename(topic, topic_types),
                    serialization_format="cdr",
                )
            )
        print("Reading from bag, timestap: ", timestamp, " topic: ", topic)
        writer.write(topic, msg, timestamp)

    del writer

    print(f"\nDone! Find your fused bag at {args.output}\n")

def typename(topic_name, topic_types):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")


if __name__ == "__main__":
    # parse args
    parser = argparse.ArgumentParser(description='fuse.py')
    parser.add_argument('--bag', help='path to input bag file')
    parser.add_argument('--mp4', help='path to directory containing input mp4 videos')
    parser.add_argument('--output', help='path to output bag file', default='/output/fused_bag.mcap')
    parser.add_argument('--topic-name', help='name of topic to write to', default='/image/postfacto/cam0')
    parser.add_argument('--raw', help='do not use jpeg compression', action='store_true', default=False)
    parser.add_argument('--jpeg-quality', help='jpeg compression quality. Disregarded if --raw == True', type=int, default=75)
    parser.add_argument('--target-fps', help='target fps for output bag', type=int, default=30)
    parser.add_argument('--target-resolution', help='target resolution for output bag. Each frame is resized to [width]x[height]', type=str, default='None')
    parser.add_argument('--video-timestamp-offset', help='offset in milliseconds to add to video timestamps. Depending on your system, there can be a ~1000ms difference between when the mp4 file is created (named) and the actual first frame. Measure it by taking a video of the system clock in ms and comparing it to the file name.', type=int, default=1000)

    args = parser.parse_args()

    if args.target_resolution != 'None':
        split_res = args.target_resolution.split('x')
        args.target_resolution = [
            int(split_res[1]),
            int(split_res[0]),
        ]

    if args.raw:
        print("WARNING: (--raw) Raw images results in hugely larger files and slower processing. Use JPEG (CompressedImage) unless you have a good reason not to.")


    main(args)
