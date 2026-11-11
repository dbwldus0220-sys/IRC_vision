# Runtime Motion Policy

- The latest source102 m forward/recovery request supersedes the earlier p/t/j
  imports below. Use `m전진*` and six `m라인복귀*` variants at source speed 1.05.
  Preserve existing repeat counts and camera variants; derive `m전진45(8회)`
  from the six-cycle source. Correct only camera motor 0 in
  `m라인복귀우회전90` to the existing 1.0 degrees in all frames (the source
  alternates -64 and 1). Preserve source body targets, torque, and timing.
  Rebuild all four goal-exit tails and embedded forward stages. In
  `n공에서 전진하고 후진준비자세`, encode the new forward at effective speed
  1.05 while preserving the preparation tail targets, duration, and gap.
  Refresh reservations and capture durations; retain public command IDs.
  Snapshot: `20261111_m_forward_recovery102/source_robot_motions.json`.

- The latest forward-only source101 request supersedes source102 j forward data.
  Use source101 `p전진*` names and original speed 1.0, with existing command
  repeat counts and camera variants. Derive `p전진45(8회)` from the six-cycle
  source; correct source `p전진0도(2회)` repeat_count=3 to the deployed count 2.
  Update embedded forward frames in both goal exits and the six-frame
  `n공에서 전진하고 후진준비자세`. Keep source101 recovery data unchanged.
  Snapshot: `20261111_forward101/source_robot_motions.json`.

- The latest recovery-only source101 request supersedes m recovery data below.
  Use original `t라인복귀우실전`, `t라인복귀우실전90도`, `t라인복귀우공용`,
  `p라인복귀좌회전45도`, `p라인복귀좌회전90도`, and `p라인복귀좌회전공용`
  at source speed 1.0 with four repeats. Rebuild all four goal-exit recovery tails
  and refresh duration estimates. Preserve source102 j forward data.
  Snapshot: `20261111_recovery101/source_robot_motions.json`.

- The latest source102 request uses `j전진45(6회)` as the body/timing
  source for all ordinary forward variants. Preserve existing repeat counts
  and per-frame camera motor 0 angles; name them `j전진*`, at source speed 1.0.
  Also update embedded forward frames in goal exits and
  `n공에서 전진하고 후진준비자세`. Keep m recovery data/speed 1.05 and
  fine gaits unchanged. Snapshot: `20261111_j_forward102/source_robot_motions.json`.

- The latest 2026-11-11 fast-forward request supersedes the source100 names
  and data below: use `m전진*` and `m라인복귀*` from
  `20261111_m_fast_forward/source_robot_motions.json`, at source speed 1.05.
  Import `r골넣기8번구역` from the same file. Derive `m전진45(8회)`
  from `m전진45(6회)` with repeat_count=8. Preserve public command IDs
  and goal-exit composite names; rebuild their gait frames and timing.

- The latest 2026-11-11 name-mapping request supersedes the stable `n` catalog
  names below: use source100 `p전진*`, `p라인복귀좌회전*`, and `t라인복귀우*`
  names. Only motion names and alias targets change; preserve frame/pose data.
  `p전진45(8회)` still derives from six-cycle source with repeat_count=8, and
  `p전진0도(2회)` retains repeat_count=2 despite the source value of 3.
  Keep public command IDs and goal-exit composite names unchanged.

- The 2026-11-11 source100 update supplies `p전진*`, `p라인복귀좌회전*`,
  and `t라인복귀우*` data under the existing `n전진*`/`n라인복귀*` catalog
  names so command aliases stay stable. Keep source speed 1.0. Derive eight
  forward cycles from the six-cycle source; the source's `p전진0도(2회)`
  has repeat_count=3, so preserve the deployed two-cycle command as 2.
  `n공에서 전진하고 후진준비자세` keeps all six supplied frames, once.
- The 2026-11-11 source99 update uses `n전진*` and `n라인복귀좌/우회전*`
  at their supplied speed 1.0. Keep command repeat counts: the source's
  `n전진45도(6회)` is mislabeled with repeat_count=4; deployed 6/8 variants
  derive from the identical four-cycle source with explicit repeat counts.
- The user's latest 2026-11-10 source89 instruction uses the **original source
  playback speed** for forward/recovery motions (1.0 in source89), superseding
  source88's 1.05. Preserve the supplied speed on reimport; do not force a constant.
  `n공확인자세` also keeps its source speed.
- For all other motions, preserve the supplied original
  `playback_speed`, except for `t허들찐`: multiply only this motion by **0.95**.
  The user's latest 2026-11-10 source83 instruction also restores original
  speed for `p공후 제자리좌회전` and `p골대후제자리좌회전`.
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
