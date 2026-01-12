import cv2
import numpy as np
from collections import deque
from paho.mqtt import publish

# ================= CAMERA =================
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -7)

TARGET_W = 640

# ================= PARAMETERS =================
MIN_AREA_ABS   = 5
MIN_AREA_RATIO = 0.000015
# new: maks area ratio (exclude very large bright regions e.g. window)
MAX_AREA_RATIO = 0.002   # ~0.2% of frame area (tweak if needed)

SCORE_THRESH   = 0.45

HSV_LOWER = np.array([8, 90, 110])
HSV_UPPER = np.array([35, 255, 255])

CR_LOWER = 135
CB_UPPER = 145
BRIGHT_MIN = 100
BRIGHT_MAX = 255

# new: saturation minimum to prefer colored (orange/yellow) flames
SAT_MIN = 70

# new: aspect ratio (height / width) min for candle-like flame
ASPECT_MIN = 1.0

# new: solidity max (exclude near-solid blobs)
SOLIDITY_MAX = 0.92

# new: flicker threshold for larger blobs
FLICKER_MIN_LARGE = 0.08

# ================= MQTT =================
# adjust with your ip, port, and mqtt topic
MQTT_BROKER = "172.20.10.2"
MQTT_PORT   = 1884
MQTT_TOPIC  = "robot/fire"

# ================= TEMPORAL =================
FRAME_HISTORY = 8
mask_history = deque(maxlen=FRAME_HISTORY)

# persistence for publishing (need majority True in last PERSISTENCE frames)
PERSISTENCE = 3
detection_history = deque(maxlen=PERSISTENCE)
for _ in range(PERSISTENCE):
    detection_history.append(0)

# ================= HELPERS =================
def resize_keep(frame, w):
    h0, w0 = frame.shape[:2]
    if w0 == w:
        return frame
    s = w / w0
    return cv2.resize(frame, (w, int(h0*s)))

def fire_color_mask(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(2.5,(8,8)).apply(l)
    frame = cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)

    color_mask  = (cr > CR_LOWER) & (cb < CB_UPPER)
    bright_mask = (y > BRIGHT_MIN) & (y < BRIGHT_MAX)

    mask = (hsv_mask > 0) & color_mask & bright_mask
    mask = (mask.astype(np.uint8)) * 255

    if np.count_nonzero(mask) < 80:
        return mask

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    return mask

def local_contrast(gray, x,y,w,h):
    pad = 6
    roi = gray[y:y+h, x:x+w]
    ring = gray[max(0,y-pad):y+h+pad, max(0,x-pad):x+w+pad]
    if roi.size == 0 or ring.size == 0:
        return 0
    return max(0, (roi.mean() - ring.mean()) / 80)

def flicker_score(history, x,y,w,h):
    if len(history) < 3:
        return 0
    diffs = []
    for i in range(1, len(history)):
        r1 = history[i-1][y:y+h, x:x+w]
        r2 = history[i][y:y+h, x:x+w]
        if r1.size == 0 or r2.size == 0:
            continue
        diffs.append(np.mean(cv2.absdiff(r1, r2)))
    return np.mean(diffs)/255 if diffs else 0

def edge_density(gray, x,y,w,h):
    roi = gray[y:y+h, x:x+w]
    if roi.size == 0:
        return 0
    edges = cv2.Canny(roi, 60, 150)
    return np.mean(edges > 0)

def entropy_score(gray, x,y,w,h):
    roi = gray[y:y+h, x:x+w]
    if roi.size == 0:
        return 0
    hist = cv2.calcHist([roi],[0],None,[32],[0,256])
    p = hist / (hist.sum() + 1e-6)
    return -np.sum(p * np.log2(p + 1e-6))

# ================= MAIN =================
prev_published_state = None

while True:
    ret, frame0 = cap.read()
    if not ret:
        break

    frame = resize_keep(frame0, TARGET_W)
    vis = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h,w = gray.shape
    MIN_AREA = max(MIN_AREA_ABS, int(h*w*MIN_AREA_RATIO))
    MAX_AREA = int(h*w*MAX_AREA_RATIO)

    mask = fire_color_mask(frame)
    mask_history.append(mask.copy())

    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected_any = False

    hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        # exclude too large bright objects (windows, lamps)
        if area > MAX_AREA:
            continue

        x,y,wc,hc = cv2.boundingRect(cnt)
        if wc == 0 or hc == 0:
            continue

        irregular = (cv2.arcLength(cnt,True)**2) / (4*np.pi*area + 1e-6)
        color_frac = area / (wc*hc + 1)

        # HSV roi checks: saturation and value (to avoid near-white windows)
        hsv_roi = hsv_full[y:y+hc, x:x+wc]
        if hsv_roi.size == 0:
            continue
        mean_sat = float(np.mean(hsv_roi[:,:,1]))
        mean_val = float(np.mean(hsv_roi[:,:,2]))
        if mean_sat < SAT_MIN:
            continue
        # aspect ratio: candle flames
        aspect = hc / (wc + 1e-6)
        if aspect < ASPECT_MIN:
            continue

        # solidity (exclude blob-like objects)
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull) if hull is not None else 0
        solidity = area / (hull_area + 1e-6)
        if solidity > SOLIDITY_MAX:
            continue

        contrast = local_contrast(gray,x,y,wc,hc)
        flicker  = flicker_score(mask_history,x,y,wc,hc)
        edges    = edge_density(gray,x,y,wc,hc)
        entropy  = entropy_score(gray,x,y,wc,hc)

        is_small_fire = area < 3 * MIN_AREA

        # HARD REJECTION tweaks
        if not is_small_fire:
            # lebih ketat untuk objek lebih besar
            if flicker < FLICKER_MIN_LARGE or edges < 0.02 or entropy < 2.2:
                continue
        else:
            if contrast < 0.15:
                continue

        # SCORING (sama logika tetapi tetap perkuat color_frac)
        if is_small_fire:
            score = (
                0.40 * color_frac +
                0.30 * min(1, contrast) +
                0.15 * min(1, irregular) +
                0.15 * min(1, entropy/3)
            )
        else:
            score = (
                0.30 * color_frac +
                0.18 * min(1, contrast) +
                0.18 * min(1, irregular) +
                0.14 * min(1, flicker*2) +
                0.10 * min(1, edges*3) +
                0.10 * min(1, entropy/4)
            )

        threshold_now = 0.32 if is_small_fire else SCORE_THRESH

        if score > threshold_now:
            detected_any = True
            color = (0,0,255) if not is_small_fire else (0,165,255)
            label = "CANDLE" if is_small_fire else "FLAME"
            cv2.rectangle(vis,(x,y),(x+wc,y+hc),color,2)
            cv2.putText(vis,f"{label} {score:.2f}",
                        (x,y-6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,color,2)
            # optionally draw extra debug info
            cv2.putText(vis,f"S:{mean_sat:.0f} V:{mean_val:.0f} A:{area}",
                        (x, y+hc+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # ===== persistence logic for MQTT publish =====
    detection_history.append(1 if detected_any else 0)
    persistent_detect = (sum(detection_history) >= (PERSISTENCE//2 + 1))  # majority

    if prev_published_state is None or persistent_detect != prev_published_state:
        payload = "ADA_API" if persistent_detect else "TIDAK_ADA_API"
        publish.single(MQTT_TOPIC, payload,
                       hostname=MQTT_BROKER, port=MQTT_PORT)
        print("MQTT:", payload)
        prev_published_state = persistent_detect

    status = "ADA_API" if persistent_detect else "TIDAK_ADA_API"
    cv2.putText(vis,status,(10,30),
                cv2.FONT_HERSHEY_SIMPLEX,1,
                (0,0,255) if persistent_detect else (0,255,0),2)

    cv2.imshow("Fire Detection (Candle Focus)", vis)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
