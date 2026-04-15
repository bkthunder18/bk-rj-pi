"""
ENME 435 – Assignment 5, Question 2
Lane Detection Pipeline — Python 3  (v2 — improved robustness)

Changes from v1:
  - Adaptive Canny thresholds based on frame brightness
  - ROI raised higher to capture actual lane lines (not just car hood)
  - Relaxed slope filter bounds (shallower lanes on highway footage)
  - Fixed left/right classification to use x-position at BOTTOM of frame
    (more reliable than x-intercept at y=0 which is off-screen)
  - Added CLAHE contrast enhancement before edge detection
  - Lowered Hough threshold to catch sparse edges
  - Added fallback: if history exists but current frame has 0 candidates,
    show history but flag it visually (yellow instead of full color)
  - Debug mode shows ROI polygon, raw Hough lines, and candidate counts
"""

import cv2
import numpy as np
from collections import deque
import sys
import os

# ─────────────────────────────────────────────────────────────────────────────
# TUNEABLE PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

# Canny — set to None for adaptive mode (recommended)
CANNY_LOW        = None        # None = auto from frame brightness
CANNY_HIGH       = None        # None = auto from frame brightness

BLUR_KERNEL      = (11, 11)      # Gaussian blur kernel

HOUGH_THRESHOLD  = 25          # Lowered: sparse edges still accumulate votes
HOUGH_RHO        = 1
HOUGH_THETA      = np.pi / 180

# Slope filter — highway lanes are shallower than city streets
MIN_SLOPE_ABS    = 0.35         # was 0.3 — catches shallower highway lanes
MAX_SLOPE_ABS    = 8.0         # was 5.0 — allow steeper lines near edges

# ROI — RAISED to capture actual lane markings (not car hood)
# Adjust ROI_TOP_FRAC lower (e.g. 0.50) if lane vanishing point is higher
ROI_TOP_FRAC        = 0.52     # top of trapezoid as fraction of frame height
ROI_TOP_WIDTH_FRAC  = 0.04     # half-width of narrow top
ROI_BOTTOM_FRAC     = 0.80     # bottom — above the car hood
ROI_BOTTOM_MARGIN   = 0.22     # inset from left/right edges at the bottom

# Temporal smoothing
SMOOTHING_FRAMES = 10

# Drawing
LANE_THICKNESS   = 10
STALE_ALPHA      = 0.45        # opacity when drawing from history with 0 candidates

# CLAHE contrast enhancement (helps a LOT on low-contrast sunset footage)
USE_CLAHE        = True
CLAHE_CLIP       = 2.0
CLAHE_TILE       = (8, 8)

# ─────────────────────────────────────────────────────────────────────────────

def adaptive_canny(gray):
    """
    Compute Canny thresholds automatically from the median pixel value.

    Formula (Otsu-inspired):
        high = median * sigma
        low  = high / 3

    This self-adjusts for bright daylight, sunset, and night footage
    without touching any constants.
    """
    if CANNY_LOW is not None and CANNY_HIGH is not None:
        return cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)

    median_val = float(np.median(gray))
    sigma      = 0.40          # controls how tight the threshold window is
    low  = int(max(0,   (1.0 - sigma) * median_val))
    high = int(min(255, (1.0 + sigma) * median_val))
    # Ensure minimum contrast spread
    high = max(high, low + 40)
    return cv2.Canny(gray, low, high)


def build_roi_mask(frame_shape):
    """
    Custom ROI for the desert-road / long straight-road video.

    This ROI is narrower than before, sits higher on the frame,
    excludes more shoulder noise, and cuts off the car hood.
    """
    h, w = frame_shape[:2]

    pts = np.array([
        [int(0.12 * w), int(0.82 * h)],  # bottom-left
        [int(0.25 * w), int(0.6 * h)],  # top-left
        [int(0.75 * w), int(0.6 * h)],  # top-right
        [int(0.9 * w), int(0.82 * h)]   # bottom-right
    ], dtype=np.int32)

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    return mask, pts


def preprocess(frame, roi_mask, clahe):
    """
    Grayscale → CLAHE (optional contrast boost) → Blur → Canny → ROI mask.

    CLAHE (Contrast Limited Adaptive Histogram Equalization) is applied
    per tile, which dramatically improves edge visibility in regions that
    are simultaneously very bright (sky) and very dark (road surface) —
    exactly the sunset scenario you have.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if USE_CLAHE and clahe is not None:
        gray = clahe.apply(gray)

    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
    edges   = adaptive_canny(blurred)
    masked  = cv2.bitwise_and(edges, edges, mask=roi_mask)
    return masked, edges  # return both for debug display


def rho_theta_to_slope_intercept(rho, theta):
    """
    Convert HoughLines (rho, theta) → slope-intercept (m, b).
    Returns (None, None) for near-vertical lines.
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    if abs(sin_t) < 1e-6:
        return None, None
    m  = -cos_t / sin_t
    x0 = rho * cos_t
    y0 = rho * sin_t
    b  = y0 - m * x0
    return m, b


def classify_lines(lines, frame_shape):
    """
    Classify Hough lines into left / right lane candidates.

    This version is stricter than before:
    - rejects shallow junk lines from road texture/cracks
    - uses x-position at bottom of ROI
    - adds a dead-zone near the image center so center cracks
      do not get misclassified as lane lines
    """
    h, w = frame_shape[:2]
    y_ref = int(0.80 * h)   # match the ROI bottom
    left_limit  = 0.45 * w
    right_limit = 0.55 * w

    left_params = []
    right_params = []

    if lines is None:
        return left_params, right_params

    for line in lines:
        rho, theta = line[0]
        m, b = rho_theta_to_slope_intercept(rho, theta)

        if m is None:
            continue

        # reject shallow / weird lines
        if abs(m) < MIN_SLOPE_ABS or abs(m) > MAX_SLOPE_ABS:
            continue

        # compute x location of the line at the lower part of the ROI
        x_at_bottom = (y_ref - b) / m

        # left lane candidate
        if m < 0 and x_at_bottom < left_limit:
            left_params.append((m, b))

        # right lane candidate
        elif m > 0 and x_at_bottom > right_limit:
            right_params.append((m, b))

    return left_params, right_params


def representative_line(params):
    """Median (m, b) from a list of candidate (slope, intercept) pairs."""
    if not params:
        return None
    ms = [p[0] for p in params]
    bs = [p[1] for p in params]
    return float(np.median(ms)), float(np.median(bs))


# ─────────────────────────────────────────────────────────────────────────────
# Temporal smoothing
# ─────────────────────────────────────────────────────────────────────────────

left_history  = deque(maxlen=SMOOTHING_FRAMES)
right_history = deque(maxlen=SMOOTHING_FRAMES)


def smooth(history, new_val):
    """
    Push new_val if valid, then return mean of buffer.
    If new_val is None (no detection this frame), history is NOT updated
    so we don't pollute the buffer with missing data.
    """
    if new_val is not None:
        history.append(new_val)
    if not history:
        return None
    ms = [v[0] for v in history]
    bs = [v[1] for v in history]
    return float(np.mean(ms)), float(np.mean(bs))


# ─────────────────────────────────────────────────────────────────────────────
# Drawing
# ─────────────────────────────────────────────────────────────────────────────

def extrapolate_line(m, b, y_top, y_bottom, frame_width):
    """Compute endpoints clamped to frame width."""
    if abs(m) < 1e-6:
        return None
    x1 = int(np.clip((y_top    - b) / m, -frame_width, 2 * frame_width))
    x2 = int(np.clip((y_bottom - b) / m, -frame_width, 2 * frame_width))
    return (x1, int(y_top)), (x2, int(y_bottom))


def draw_lane_line(frame, m, b, y_top, y_bottom, color, stale=False):
    """
    Draw a lane line on the frame.
    If stale=True (drawing from history with no live candidates),
    the line is drawn semi-transparent and thinner as a visual cue.
    """
    result = extrapolate_line(m, b, y_top, y_bottom, frame.shape[1])
    if result is None:
        return

    pt1, pt2 = result
    if stale:
        # Draw on overlay and blend to indicate uncertainty
        overlay = frame.copy()
        cv2.line(overlay, pt1, pt2, color, LANE_THICKNESS // 2, cv2.LINE_AA)
        cv2.addWeighted(overlay, STALE_ALPHA, frame, 1 - STALE_ALPHA, 0, frame)
    else:
        cv2.line(frame, pt1, pt2, color, LANE_THICKNESS, cv2.LINE_AA)


def draw_roi_overlay(frame, roi_pts):
    """Draw the ROI trapezoid as a faint overlay for tuning."""
    overlay = frame.copy()
    cv2.polylines(overlay, [roi_pts], isClosed=True, color=(0, 220, 255), thickness=2)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)


def draw_hough_lines_debug(frame, lines, frame_shape):
    """Draw all raw Hough lines in the ROI vertical range for debugging."""
    if lines is None:
        return

    h, w = frame_shape[:2]
    y1, y2 = int(0.55 * h), int(0.80 * h)

    for line in lines:
        rho, theta = line[0]
        m, b = rho_theta_to_slope_intercept(rho, theta)
        if m is None:
            continue

        result = extrapolate_line(m, b, y1, y2, w)
        if result:
            cv2.line(frame, result[0], result[1], (255, 200, 0), 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def process_video(video_path, show_debug=True):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open: {video_path}")
        return

    ret, first_frame = cap.read()
    if not ret:
        print("[ERROR] Cannot read first frame.")
        cap.release()
        return

    roi_mask, roi_pts = build_roi_mask(first_frame.shape)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    h = first_frame.shape[0]
    w = first_frame.shape[1]
    y_bottom = int(0.80 * h)
    y_top    = int(0.55 * h)

    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE) if USE_CLAHE else None

    frame_num   = 0
    print(f"[INFO] Processing: {video_path}  |  Press Q to quit, D to toggle debug, R to reset history")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[INFO] End of video.")
            break
        frame_num += 1

        edges_masked, edges_full = preprocess(frame, roi_mask, clahe)

        lines = cv2.HoughLines(edges_masked, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD)

        left_params, right_params = classify_lines(lines, frame.shape)

        left_rep  = representative_line(left_params)
        right_rep = representative_line(right_params)

        left_smooth  = smooth(left_history,  left_rep)
        right_smooth = smooth(right_history, right_rep)

        # ── Draw ─────────────────────────────────────────────────────────────
        output = frame.copy()

        if show_debug:
            draw_roi_overlay(output, roi_pts)
            draw_hough_lines_debug(output, lines, frame.shape)

        left_stale  = (left_rep  is None and left_smooth  is not None)
        right_stale = (right_rep is None and right_smooth is not None)

        if left_smooth:
            draw_lane_line(output, *left_smooth, y_top, y_bottom,
                           color=(0, 0, 255), stale=left_stale)

        if right_smooth:
            draw_lane_line(output, *right_smooth, y_top, y_bottom,
                           color=(0, 255, 0), stale=right_stale)

        # ── HUD ──────────────────────────────────────────────────────────────
        stale_flag = " [STALE]" if (left_stale or right_stale) else ""
        cv2.putText(output,
                    f"Frame {frame_num} | L: {len(left_params)} | R: {len(right_params)}{stale_flag}",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 0) if stale_flag else (255, 255, 255),
                    2, cv2.LINE_AA)

        cv2.putText(output, "D=debug  R=reset  Q=quit",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (180, 180, 180), 1, cv2.LINE_AA)

        cv2.imshow("Lane Detection", output)

        if show_debug:
            # Stack edges side-by-side: raw Canny | ROI-masked Canny
            edges_full_rgb   = cv2.cvtColor(edges_full,   cv2.COLOR_GRAY2BGR)
            edges_masked_rgb = cv2.cvtColor(edges_masked, cv2.COLOR_GRAY2BGR)
            debug_panel = np.hstack([edges_full_rgb, edges_masked_rgb])
            # Scale down so it doesn't dwarf the main window
            dh = h // 2
            dw = int(debug_panel.shape[1] * dh / debug_panel.shape[0])
            debug_panel = cv2.resize(debug_panel, (dw, dh))
            cv2.putText(debug_panel, "Full Canny  |  ROI-masked Canny",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow("Edges", debug_panel)

        key = cv2.waitKey(25) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            show_debug = not show_debug
            if not show_debug:
                cv2.destroyWindow("Edges")
        elif key == ord('r'):
            left_history.clear()
            right_history.clear()
            print(f"[INFO] History reset at frame {frame_num}")

    cap.release()
    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    videos = sys.argv[1:] if len(sys.argv) > 1 else ["20190130-073331.mp4"]

    for v in videos:
        if not os.path.exists(v):
            print(f"[WARN] Not found, skipping: {v}")
            continue
        left_history.clear()
        right_history.clear()
        process_video(v, show_debug=True)

    print("Done.")


# ─────────────────────────────────────────────────────────────────────────────
# TUNING GUIDE  — read this if lines still look wrong
# ─────────────────────────────────────────────────────────────────────────────
"""
PROBLEM: 0 candidates on both sides
  → Lower HOUGH_THRESHOLD (try 10)
  → Lower MIN_SLOPE_ABS (try 0.15)
  → Raise ROI_TOP_FRAC (try 0.65) so ROI captures more road
  → Check the "Full Canny" debug panel — if it's mostly black,
    lower CANNY_LOW/HIGH manually (try 20/60)

PROBLEM: Lines on wrong side / crossing
  → The x_at_bottom classification is the most reliable fix.
    Already implemented in v2.  If still wrong, print the slopes:
    add  print(f"L slopes: {[p[0] for p in left_params]}")
    and verify they are negative.

PROBLEM: Lines jittering / jumping
  → Increase SMOOTHING_FRAMES (try 15–20)
  → Raise HOUGH_THRESHOLD (reduces noisy detections feeding the buffer)

PROBLEM: Right lane only, no left (or vice versa)
  → The left lane marking in this video is a dashed line — Canny may
    only catch one dash at a time, giving fewer votes.
    Lower HOUGH_THRESHOLD further (try 8) for that side.

PROBLEM: Car hood being detected as a line
  → Lower ROI_BOTTOM_FRAC from 0.92 to 0.85

PROBLEM: Retaining wall / guardrail detected on right
  → The wall on the right of your video is very high-contrast.
    Narrow the ROI by increasing ROI_BOTTOM_MARGIN (try 0.12).
"""