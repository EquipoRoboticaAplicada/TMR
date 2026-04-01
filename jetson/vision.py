# vision_common.py
import json
import cv2 as cv
import numpy as np

DRAW = {
    "green": (0, 255, 0),
    "blue":  (255, 0, 0),
    "red":   (0, 0, 255)
}

MIN_AREA = 600
KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))


def load_color_ranges(color_file="colors.json"):
    with open(color_file, "r") as f:
        return json.load(f)


def process_mask(mask):
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, KERNEL, iterations=1)
    return mask


def find_and_draw(mask, frame_draw, label, draw=True):
    found = False
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    detected_centroids = []
    areas = []

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > MIN_AREA:
            found = True
            areas.append(area)

            M = cv.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_centroids.append((label, cx, cy))

                if draw:
                    x, y, w, h = cv.boundingRect(cnt)
                    color_bgr = DRAW.get(label, (255, 255, 255))
                    cv.rectangle(frame_draw, (x, y), (x + w, y + h), color_bgr, 2)
                    cv.putText(frame_draw, label, (x, y - 8),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2, cv.LINE_AA)
                    cv.circle(frame_draw, (cx, cy), 5, (255, 255, 255), -1)

    return found, detected_centroids, areas


def detect_colors(frame, color_ranges, draw=False):
    colors = []
    centroids = []
    areas = []

    blurred = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    for color, ranges in color_ranges.items():
        if isinstance(ranges, dict):
            lower = np.array(ranges["lower"], dtype=np.uint8)
            upper = np.array(ranges["upper"], dtype=np.uint8)
            mask = cv.inRange(hsv, lower, upper)

        elif isinstance(ranges, list):
            mask = None
            for r in ranges:
                lower = np.array(r["lower"], dtype=np.uint8)
                upper = np.array(r["upper"], dtype=np.uint8)
                m = cv.inRange(hsv, lower, upper)
                mask = m if mask is None else cv.bitwise_or(mask, m)

            if mask is None:
                continue
        else:
            continue

        mask = process_mask(mask)

        found, c_list, a_list = find_and_draw(mask, frame, color, draw)

        if found:
            colors.extend([color] * len(c_list))
            centroids.extend(c_list)
            areas.extend(a_list)

    return colors, centroids, areas


def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas:
        return None
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates:
        return None
    centroid, area = max(candidates, key=lambda x: x[1])
    return centroid, area
