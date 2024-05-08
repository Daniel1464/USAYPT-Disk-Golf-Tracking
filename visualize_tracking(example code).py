import sys
import cv2 as cv

# set video file path and tracker type here
# no idea why mil is the only type of tracker
video = cv.VideoCapture("")
tracker = cv.TrackerMIL()
# bounding box
bbox = (0, 0, 0, 0)


if not video.isOpened():
    print("Video has not been opened; stopping program")
    sys.exit()

while True:
    # Read a new frame
    ok, frame = video.read()
    if not ok:
        break

    # Start timer
    timer = cv.getTickCount()

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Calculate Frames per second (FPS)
    fps = cv.getTickFrequency() / (cv.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1: tuple[int, int] = (int(bbox[0]), int(bbox[1]))
        p2: tuple[int, int] = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    else:
        # Tracking failure
        cv.putText(frame, "Tracking failure detected", (100, 80), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display tracker type on frame
    cv.putText(frame, str(tracker) + " Tracker", (100, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display FPS on frame
    cv.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display result
    cv.imshow("Tracking", frame)
    

    # Exit if ESC pressed
    if cv.waitKey(1) & 0xff == 27:
        break

video.release()
cv.destroyAllWindows()
