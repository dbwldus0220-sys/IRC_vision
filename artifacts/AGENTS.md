# Runtime Motion Policy

- On every motion replacement/import, set runtime `playback_speed` to the
  supplied original speed multiplied by **0.95**, for every motion family.
  This standing user instruction was reaffirmed on 2026-10-29.
- Keep `robot_motions_pc.json` and archived input files at the original speed.
  Derive repeat/camera variants from that unscaled source, then apply 0.95 once.
  Never multiply the existing runtime speed again or exempt fine/turn/crab motions.
- Use `tools/upsert_motion_catalog.py:runtime_motion_from_source` for imports.
  Reimporting the same source must produce identical data, not cumulative slowing.
  Its `_with_runtime_policy` helper only normalizes tolerance/pose identities and
  must not be used alone to import source motions.
- Update motion-duration/capture/prequeue estimates when source timing or runtime
  speed changes. Verify against the actual generated runtime catalog.

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
