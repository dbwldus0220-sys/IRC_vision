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

- In every frame of every runtime motion, override motor ID 4 to 18 degrees and
  motor ID 5 to -18 degrees, except for these exact motion names:
  `찐공잡기리그랩까지 실전`, `찐골넣기`, and `찐허들`.
- Preserve motors 4 and 5 as supplied for those exceptions. Do not exempt other
  pickup, retreat, or goal-related motions based on a name substring.
- Apply this fixed-joint policy on imports, replacements, and manual updates.
  Preserve other joint targets, torque flags, timing, speed, and repeat counts.
  Archived source snapshots must remain unchanged.
- `tools/upsert_motion_catalog.py` applies both runtime policies. Verify imports
  and the current catalog with `python3 -m pytest -q tools/test_upsert_motion_catalog.py`.
