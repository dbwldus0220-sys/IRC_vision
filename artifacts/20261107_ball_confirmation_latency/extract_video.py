#!/usr/bin/env python3
"""Extract screen evidence by PTS; video FPS metadata is not a frame clock."""
from pathlib import Path
import cv2

OUT = Path(__file__).resolve().parent
VIDEO = Path('/home/jet/Videos/Screencasts/Screencast from 2026년 11월 07일 18시 36분 18초.webm')
TARGETS = {
    79213: 'screen_79.213_candidate9.jpg',
    79514: 'screen_79.514_candidate10.jpg',
    79597: 'screen_79.597_candidate10_but_log_confirmed.jpg',
    79857: 'screen_79.857_candidate11_but_log_confirmed.jpg',
    80476: 'screen_80.476_confirmed.jpg',
}
cap = cv2.VideoCapture(str(VIDEO))
found = set()
while cap.grab():
    pts_ms = round(cap.get(cv2.CAP_PROP_POS_MSEC))
    if pts_ms in TARGETS:
        ok, frame = cap.retrieve()
        if not ok:
            raise RuntimeError(f'Cannot decode frame at {pts_ms}')
        if not cv2.imwrite(str(OUT/TARGETS[pts_ms]), frame):
            raise RuntimeError('Cannot save evidence')
        found.add(pts_ms)
    if pts_ms > max(TARGETS):
        break
cap.release()
assert found == set(TARGETS), f'Missing PTS: {set(TARGETS)-found}'
print(f'Extracted {len(found)} screen evidence frames. No inference or hardware used.')
