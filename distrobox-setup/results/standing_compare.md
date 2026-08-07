# Standing comparison — MuJoCo vs Isaac (cmd_vel = 0)

Date: 2026-07-18. Same policy (neural_controller config.yaml -> policy_latest.json),
same kinematics, backlash OFF both, latency 8/10 both. Robot standing, zero command.

## Isaac — STATIC (locks instantly)
std ~1e-4 rad over 6s, body_z 0.17461 (spread 1e-5). Reached immediately.

    joint          rad
    leg_front_r_1   1.03822
    leg_front_r_2  -0.19903
    leg_front_r_3  -1.29060
    leg_front_l_1  -0.94757
    leg_front_l_2   0.17874
    leg_front_l_3   1.30430
    leg_back_r_1    0.21049
    leg_back_r_2    0.01284
    leg_back_r_3   -1.46928
    leg_back_l_1   -0.33918
    leg_back_l_2   -0.17704
    leg_back_l_3    1.36966

## MuJoCo — SLOW CREEP, plateaus ~2min (UPDATED)
Later run: reaches steady state after ~2min (t=30/60/90 identical: fr_1 1.0725, fl_2 -0.170,
br_2 0.291...). Isaac settles instantly, MuJoCo ~2min. Plateau pose diff from Isaac is LARGER
than mid-transient: hip _2 joints off 16-20deg (fl_2 20deg, br_2 16deg), RMS ~9deg. Real
steady-state gap, not a settling artifact.

## MuJoCo — original 90s creep trace
Base orientation stable (quat w~0.995, yaw ~-11deg, unchanged). Joints drift
monotonically -> robot slowly creeps in place; policy compensates. std within each 6s
window low (~1e-3) so no fast oscillation, just slow mean drift.

Trajectory (window mean, canonical order fr1,fr2,fr3,fl1,fl2,fl3,br1,br2,br3,bl1,bl2,bl3):

    t=5.0   1.026 -0.212 -1.388 -0.947  0.217 1.362  0.281 0.110 -1.454 -0.415 -0.303 1.388
    t=30.0  1.031 -0.207 -1.359 -0.974  0.183 1.351  0.254 0.106 -1.459 -0.400 -0.322 1.383
    t=60.1  1.035 -0.201 -1.317 -0.995  0.135 1.334  0.224 0.113 -1.460 -0.377 -0.331 1.380
    t=85.1  1.037 -0.200 -1.281 -1.005  0.091 1.318  0.205 0.131 -1.460 -0.354 -0.325 1.378

Still moving at 85s (fl_2 0.217->0.091 and falling; fr_3 -1.388->-1.281 and rising).

## Diff Isaac vs MuJoCo@85s
Biggest: bl_2 8.5deg, br_2 6.8deg, fl_2 5.0deg, fl_1 3.3deg. RMS ~3.6deg (NOT converged,
MuJoCo still drifting). Some joints did converge to Isaac (fr_1 1.037 vs 1.038; br_1 0.205
vs 0.210).

## Headline
Same policy => STATIC equilibrium in Isaac, SLOW CONTINUOUS CREEP in MuJoCo. This is the
main sim-to-sim behavioral gap, bigger than any single joint offset.

## Caveats / not-yet-done
- MuJoCo body_z NOT measured (no world-frame topic; mujoco_sensor/body_pos is an unbroadcast
  state interface). Standing confirmed upright via IMU only.
- MuJoCo not at steady state -> pose diff is a moving target.
- Physics rate differs: Isaac 250Hz vs MuJoCo 10kHz (documented, not equalized).

## Candidate causes of the creep (untested)
- Contact friction creep / foot sliding in MuJoCo.
- Physics-rate-dependent numerical damping (Isaac 250Hz PhysX damps the slow mode;
  MuJoCo 10kHz does not).
- A slow closed-loop mode of the policy that only one engine's numerics suppress.

## Next steps
- Root-cause the creep: log MuJoCo base translation (need a body_pos broadcaster) to see if
  feet slide; try matching friction/solver more tightly.
- Longer MuJoCo run (5+ min) to see if it ever plateaus.
- Then the forward-walk comparison.
