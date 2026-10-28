# Runtime Motion Policy

- Every motion in `robot_motions_runtime.json` must use
  `completion.position_tolerance_deg = 5.0`, including hurdle motions.
- Apply this policy on every JSON replacement, import, and manual motion update,
  regardless of the tolerance in the supplied source JSON.
- Preserve archived source snapshots unchanged. Normalize only runtime data.
- Do not change joint targets, torque flags, timing, speed, or repeat counts when
  applying the tolerance policy.
- `tools/upsert_motion_catalog.py` already normalizes imported and existing runtime
  motions to 5.0. For a manual partial import, apply the same policy and run
  `test_production_motions_keep_approved_final_tolerances` in
  `src/mission_control/test/test_production_motion_catalog_contract.py`.

- Preserve source motor ID 4 and 5 values in every motion, including startup
  poses. The user revoked the 18/-18 degree fixed-shoulder policy on 2026-10-27.
  Do not reintroduce it on import, replacement, manual update, or normal playback.
- `robot_motions_pc.json` preserves the original shoulder targets for the
  current deployed catalog. Match by motion and frame identity when restoring
  runtime shoulder targets; preserve all other fields and archived snapshots.
- Keep the existing `(건공잡기)` runtime pose-name suffixes for compatibility.
- `tools/upsert_motion_catalog.py` normalizes completion tolerance and preserves
  supplied joint targets. Verify with
  `python3 -m pytest -q tools/test_upsert_motion_catalog.py`.
