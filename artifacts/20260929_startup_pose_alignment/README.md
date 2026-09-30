# Startup pose alignment

- Reference: `찐찐전진(6회)` / `오뒤412` (not the old single-찐 forward motion).
- Excluded motion joint data: `찐공잡기리그랩까지 실전`, `찐골넣기`, `찐허들`.
- All 92 non-excluded frames whose names contain `오뒤` retain motors 4=18 and 5=-18.
- All 30 ordinary `오뒤412` frames now share the reference angles. In 24 frames, motor 13 changed from -43.59375 to -41.59375 and motor 14 from 57.568359375 to 55.568359375.
- Hurdle joint targets are unchanged. Its distinct terminal frame and end_pose are named `오뒤412(허들)` to keep startup selection unambiguous.
- Camera-specific and fine/backward pose variants retain other joint targets; timing, torque flags, speeds, and repetitions are unchanged.
- Backup: `runtime_before.json`; full changes: `manifest.json`.
- Verification: 12 Python cases passed. The actual C++ `load_startup_pose_angles` was compiled as a hardware-free probe and successfully loaded 23 joints from both the source and installed runtime catalogs. Catalog validation passed for all 89 motions.
- No robot hardware was accessed. Installed runtime is a symlink to the source file. Restart the executor to reload data; simulate pose transitions before hardware operation.
