"""Compare the staged TensorRT engine with ONNX on saved camera frames."""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

root = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(root / 'src/step'))
from step.tensorrt_backend import TensorRTBackend

folder = Path(__file__).resolve().parent
session = ort.InferenceSession(str(folder / 'new.best.onnx'), providers=['CPUExecutionProvider'])
backend = TensorRTBackend(folder / 'new.best.engine')
report = {'input_shape': backend.input_shape, 'output_shape': backend.output_shape, 'frames': []}
assert backend.input_shape == (1, 3, 640, 640)
assert backend.output_shape == (1, 300, 6)
try:
    for name in ['frame_104.102.jpg', 'frame_170.013.jpg', 'frame_155.061.jpg']:
        path = root / 'artifacts/20260927_000626_review' / name
        image = cv2.imread(str(path))
        assert image is not None, str(path)
        height, width = image.shape[:2]
        scale = min(640 / width, 640 / height)
        rw, rh = max(1, round(width * scale)), max(1, round(height * scale))
        resized = cv2.resize(image, (rw, rh), interpolation=cv2.INTER_LINEAR)
        pw, ph = 640 - rw, 640 - rh
        padded = cv2.copyMakeBorder(resized, ph // 2, ph - ph // 2, pw // 2, pw - pw // 2, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        blob = np.ascontiguousarray(cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
        reference = session.run(None, {session.get_inputs()[0].name: blob})[0]
        backend.infer(blob)
        start = time.perf_counter()
        actual = backend.infer(blob).copy()
        elapsed_ms = (time.perf_counter() - start) * 1000
        assert actual.shape == reference.shape
        assert np.isfinite(actual).all()
        assert ((actual[..., 4] >= 0) & (actual[..., 4] <= 1)).all()
        detections = actual[0][actual[0, :, 4] >= 0.25]
        expected = reference[0][reference[0, :, 4] >= 0.25]
        assert len(detections) == len(expected), (name, len(detections), len(expected))
        candidates = list(expected)
        max_box_error = max_conf_error = 0.0
        for row in detections:
            matches = [(i, float(np.max(np.abs(row[:4] - candidate[:4])))) for i, candidate in enumerate(candidates) if int(row[5]) == int(candidate[5])]
            assert matches, (name, 'class mismatch')
            index, box_error = min(matches, key=lambda item: item[1])
            match = candidates.pop(index)
            conf_error = float(abs(row[4] - match[4]))
            assert box_error < 2.0 and conf_error < 0.02, (name, box_error, conf_error)
            max_box_error = max(max_box_error, box_error)
            max_conf_error = max(max_conf_error, conf_error)
        report['frames'].append({'image': str(path), 'detections_above_0.25': len(detections), 'max_box_difference_pixels': max_box_error, 'max_confidence_difference': max_conf_error, 'inference_ms_single_sample': elapsed_ms})
finally:
    backend.close()
(folder / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
