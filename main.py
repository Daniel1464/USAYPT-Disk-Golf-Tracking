import sys
from typing import Sequence
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
#TODO: Use trig to determine tilt of disk, find a way to calc rotational velocity of disk, maybe find a way to use multiple cameras?

# the starting bounding box of the disc; fill this out!
STARTING_BOUNDING_BOX = (0, 0, 0, 0)
# the diameter of the disc in meters; fill this out!
DISC_DIAMETER_METERS = 0.0


def get_delta(bbox_current: Sequence[int], bbox_previous: Sequence[int]) -> tuple[int, int, int, int]:
    return (
        bbox_current[0] - bbox_previous[0],
        bbox_current[1] - bbox_previous[1],
        bbox_current[2] - bbox_previous[2],
        bbox_current[3] - bbox_previous[4]
    )


# set video file path and tracker type here
# no idea why mil is the only type of tracker
video = cv.VideoCapture("", cv.CAP_DSHOW)
tracker = cv.TrackerMIL()
if not video.isOpened():
    print("Video has not been opened; stopping program")
    sys.exit()


ok, starting_frame = video.read()
if not ok:
    print("Could not read first frame; stopping program")
    sys.exit()
tracker.init(starting_frame, STARTING_BOUNDING_BOX)


seconds_per_frame: float = 1.0 / video.get(cv.CAP_PROP_FPS)
num_frames: int = 0
previous_bbox: Sequence[int] = STARTING_BOUNDING_BOX


timestamps = np.array([])
velocity_values = np.array([])


while True:
    num_frames += 1
    # Read a new frame
    ok, frame = video.read()
    if not ok:
        print("Stopped processing frame/could not process frame.")
        break

    # Update tracker
    ok, current_bbox = tracker.update(frame)
    if not ok:
        break

    # determines the shift in pixels between frames
    bounding_box_delta = get_delta(current_bbox, previous_bbox)
    # use this
    pixel_shift_per_sec = (corner_delta * seconds_per_frame for corner_delta in bounding_box_delta)

    # fetches the disc diameter in pixels
    disc_diameter_pixels = current_bbox[2]
    # calculates distance of 1 pixel in real life(in meters)
    pixel_distance_meters = DISC_DIAMETER_METERS / disc_diameter_pixels
    disc_velocity_estimate_mps = sum(pixel_shift * pixel_distance_meters for pixel_shift in pixel_shift_per_sec) / 4.0

    np.append(timestamps, seconds_per_frame * num_frames)
    np.append(velocity_values, disc_velocity_estimate_mps)
    previous_bbox = current_bbox

video.release()

plt.plot(timestamps, velocity_values)
plt.show()

cv.destroyAllWindows()
