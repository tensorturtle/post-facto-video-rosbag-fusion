# Use NVIDIA Jetson's built-in NVENC encoder to record video from the CSI camera
# Automatically and seamlessly split video every 1 minute into different files without losing frames using splitmuxsink instead of usual filesink

# If one camera is connected via CSI, default is 0.
# If two cameras are connected via CSI, set to 0 or 1.
SENSOR_ID=0

# Get time in milliseconds
TIME_MS=$(date +%s%3N)

# Set the output video file
OUTPUT_DIR="../output/video"
OUTPUT_FILE_PREFIX="csi-$TIME_MS-ms-"
OUTPUT_FILE_SUFFIX="-minutes"

mkdir -p $OUTPUT_DIR


# Use `v4l2-ctl --device /dev/video0 --list-formats-ext` to find out below info:
VIDEO_WIDTH=1640
VIDEO_HEIGHT=1232
VIDEO_FRAMERATE=30

# Set the video bitrate
VIDEO_BITRATE=8000000

# Flip method
# 0 - no rotation
# 1 - counterclockwise 90 degrees
# 2 - rotate 180 degrees
# 3 - clockwise 90 degrees
# 4 - flip horizontally
# 5 - upper right diagonal flip
# 6 - flip vertically
# 7 - upper left diagonal flip
FLIP_METHOD=0


# Start recording using gstreamer
# split video every 1 minute into different files
# gstreamer command
gst-launch-1.0 \
    nvarguscamerasrc sensor_id=$SENSOR_ID ! \
    'video/x-raw(memory:NVMM), width=(int)'$VIDEO_WIDTH', height=(int)'$VIDEO_HEIGHT', format=(string)NV12, framerate=(fraction)'$VIDEO_FRAMERATE'/1' ! \
    nvvidconv flip-method=$FLIP_METHOD ! \
    nvv4l2h264enc maxperf-enable=1 bitrate=$VIDEO_BITRATE ! \
    h264parse ! \
    splitmuxsink location=$OUTPUT_DIR/$OUTPUT_FILE_PREFIX%03d$OUTPUT_FILE_SUFFIX.mp4 max-size-time=60000000000 -e


