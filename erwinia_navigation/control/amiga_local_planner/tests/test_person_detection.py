#!/usr/bin/env python3
"""
Unit test for YOLO person detection — mirrors obstacle_detector._check_human logic.

Usage:
  python3 test_person_detection.py               # live camera
  python3 test_person_detection.py image.jpg     # static image
"""
import sys
from collections import deque
import cv2
from ultralytics import YOLO

# Must match obstacle_detector.py exactly
YOLO_MODEL        = 'yolov8n.pt'
HUMAN_CONF_THRESH = 0.5
PERSON_CLASS_ID   = 0
DETECT_WINDOW     = 10
DETECT_MIN_FRAMES = 5
CONFIRM_RATIO     = 0.6
CLEAR_RATIO       = 0.2


def check_human(model, frame, detect_window, human_confirmed):
    """Same logic as ObstacleDetector._check_human."""
    results = model(frame, verbose=False)
    person_this_frame = any(
        int(box.cls[0]) == PERSON_CLASS_ID and float(box.conf[0]) >= HUMAN_CONF_THRESH
        for result in results
        for box in result.boxes
    )
    detect_window.append(person_this_frame)

    if len(detect_window) < DETECT_MIN_FRAMES:
        return human_confirmed, person_this_frame, results

    ratio = sum(detect_window) / len(detect_window)

    if not human_confirmed and ratio >= CONFIRM_RATIO:
        human_confirmed = True
        print(f'[HUMAN CONFIRMED] ratio={ratio:.2f}')
    elif human_confirmed and ratio < CLEAR_RATIO:
        human_confirmed = False
        print(f'[HUMAN CLEARED]   ratio={ratio:.2f}')

    return human_confirmed, person_this_frame, results


def draw(frame, results, human_confirmed):
    for result in results:
        for box in result.boxes:
            if int(box.cls[0]) == PERSON_CLASS_ID and float(box.conf[0]) >= HUMAN_CONF_THRESH:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'person {conf:.2f}', (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    status = 'STOP (human confirmed)' if human_confirmed else 'clear'
    color  = (0, 0, 255) if human_confirmed else (200, 200, 200)
    cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    return frame


def run_static(model, path):
    frame = cv2.imread(path)
    if frame is None:
        print(f'ERROR: cannot read {path}')
        sys.exit(1)

    detect_window   = deque(maxlen=DETECT_WINDOW)
    human_confirmed = False

    # Feed the same frame repeatedly to fill the window (simulates video stream)
    for i in range(DETECT_WINDOW):
        human_confirmed, person_this_frame, results = check_human(
            model, frame, detect_window, human_confirmed)
        ratio = sum(detect_window) / len(detect_window) if detect_window else 0.0
        print(f'frame {i+1:2d}: person_seen={person_this_frame}  '
              f'window={list(detect_window)}  ratio={ratio:.2f}  confirmed={human_confirmed}')

    cv2.imshow('result', draw(frame, results, human_confirmed))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_live(model):
    import pyrealsense2 as rs
    import numpy as np

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print('RealSense started. Press Q to quit.')

    detect_window   = deque(maxlen=DETECT_WINDOW)
    human_confirmed = False

    try:
        while True:
            frames      = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            human_confirmed, person_this_frame, results = check_human(
                model, frame, detect_window, human_confirmed)
            ratio = sum(detect_window) / len(detect_window) if detect_window else 0.0
            print(f'\rperson={person_this_frame}  ratio={ratio:.2f}  '
                  f'confirmed={human_confirmed}   ', end='', flush=True)

            cv2.imshow('person detection', draw(frame, results, human_confirmed))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    print(f'Loading {YOLO_MODEL} ...')
    model = YOLO(YOLO_MODEL)
    print('Model loaded')

    if len(sys.argv) > 1:
        run_static(model, sys.argv[1])
    else:
        run_live(model)
