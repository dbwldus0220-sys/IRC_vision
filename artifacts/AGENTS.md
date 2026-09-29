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
