# Runtime Motion Policy

- On every motion replacement/import, preserve the supplied original
  `playback_speed`, except for `t허들찐`, `p공후 제자리좌회전`, and
  `p골대후제자리좌회전`: multiply only these three by **0.95**.
  The user's 2026-11-10 instruction supersedes the previous all-motion scaling.
- Keep `robot_motions_pc.json` and archived input files at the original speed.
  Derive repeat/camera variants from that unscaled source, then apply the named
  exceptions once. Never multiply an existing runtime speed again.
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
