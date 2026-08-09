# Intersection Controller v3 — OCBF + Dynamic Resequencing

A new control approach for the signal-free 12-path intersection simulator,
combining two methods:

- **OCBF control** (Paper 1 — Xu, Xiao, Cassandras, Zhang, Li, *A General
  Framework for Decentralized Safe Optimal Control of CAVs in Multi-Lane
  Signal-Free Intersections*, IEEE T-ITS 2022): each CAV tracks an unconstrained
  energy-and-time-optimal **reference** trajectory with an **Optimal-Control +
  Control-Barrier-Function** QP that enforces all safety constraints online.
- **Dynamic resequencing** (Paper 2 — Zhang & Cassandras, *A Decentralized
  Optimal Control Framework for CAVs at Urban Intersections with Dynamic
  Resequencing*, IEEE CDC 2018): when a CAV enters the control zone, the crossing
  order is re-evaluated to improve throughput; that order defines the lateral
  conflict priorities (who yields to whom).

This is a **separate approach** from the reorganized v2 controller — both v2 and
the original folder are left untouched.

## How to run

1. Open MATLAB in this folder.
2. Run `MAIN`. It puts the tree on the path, configures parameters, builds the
   coordinator and vehicle population, and animates the simulation.

Requires the Optimization Toolbox (`quadprog` for the OCBF QP, `fsolve` for the
reference solve).

## Method, and how it maps to the two papers

### 1. Reference trajectory — Paper 1, eqs. (11)–(14)

When CAV *i* enters the control zone, `plan_unconstrained_reference` solves the
**unconstrained** decentralized optimal control problem with **free terminal
time**, minimizing `beta*(tm - t0) + integral 1/2 u^2 dt`. The `beta` term is
exactly "time in the objective." The five algebraic conditions (eq. 14) are
solved with `fsolve` for `[a, b, c, d, tm]`, giving

```
u*(t) = a t + b,   v*(t) = 1/2 a t^2 + b t + c,   x*(t) = 1/6 a t^3 + 1/2 b t^2 + c t + d.
```

This is computed **once** per CAV. It is never replanned on resequencing (see
below), exactly as Paper 1 §V.E prescribes for OCBF+DR.

### 2. OCBF controller — Paper 1, eqs. (16)–(19)

Every step, `ocbf_controller` solves the QP (eq. 17)

```
min over (u, e):  1/2 (u - u_ref)^2 + clf_weight * e^2
```

subject to linear constraints in the decision `z = [u; e]`:

- **rear-end CBF** (eq. 18) with the same-lane predecessor `Preceding`;
- **lateral CBFs** (eq. 19), one per duty in `ConflictSet`, using the linear
  `Phi(x_i) = phi * x_i / x_i(t_k)` barrier form;
- **speed-limit CBFs** for `vmax` / `vmin`;
- **CLF speed tracking** (eq. 16), soft via the relaxation `e`;
- control bounds `umin <= u <= umax`.

All constraints are linear in `z`, so each call is a single `quadprog`. The
controller is now **wired in** (the v2 CBF code was dormant). On QP infeasibility
it falls back to the clamped reference acceleration.

### 3. Dynamic resequencing — Paper 2, §III

`dynamic_resequencing` maintains `coordinator.S`, the crossing order. On each
arrival, CAV *i* is inserted at the feasible position minimizing the
crossing-time spread, while (a) preserving the relative order of existing CAVs
and (b) never being placed ahead of its same-lane predecessor (the no-overtaking
rule that bounds complexity to roughly the number of lanes, Paper 2 §IV).

### 4. Order -> lateral priorities — Paper 1, Algorithm 2

`determine_conflict_sets` walks `coordinator.S` and, for each CAV and each
conflict node on its path, finds the **closest higher-priority** CAV on a
different path that shares that node. That pair `(mp, j)` becomes a lateral duty
in `CAVs(i).ConflictSet`. Conflicts with farther higher-priority CAVs are
satisfied transitively. Same-lane interactions are excluded here (handled by the
rear-end CBF). Conflict sets are recomputed (cheaply) on every arrival and
departure — trajectories are **not** recomputed.

### Per-step data flow (`update_cavs`)

```
Pass 1: refresh every active CAV's conflict-node distances/times (snapshot)
Pass 2: per CAV:
          outside CZ            -> coast straight
          first entry           -> add_preceding_vehicles
                                   plan_unconstrained_reference   (eq. 14)
                                   update_arrival_times_...        (register MPs)
                                   dynamic_resequencing            (insert into S)
                                   determine_conflict_sets         (S -> duties)
          every step in CZ      -> u_ref, v_ref from the reference
                                   ocbf_controller                (the QP)
                                   cav_steering_update            (bicycle steer)
                                   integrate bicycle kinematics
          leaving CZ (cleared)  -> mark Done, drop from S, refresh conflict sets
```

## Assumptions and design decisions

These are deliberate adaptations of the two papers onto your existing simulator;
they are the things most worth checking against your intent.

1. **Existing 12-path geometry, no lane-changing.** Paper 1 uses a 2-lane-per-
   approach model (lanes l1–l8) with lane-changing and "floating" MPs. This
   simulator has 12 fixed paths and no lane changing, so the *methods* are ported
   onto that geometry; floating MPs, the coordinate transformation (eq. 10), and
   lane-change MP placement (eqs. 8–9) are **not** implemented.
2. **Your physical scale is kept.** Control zone ~100–200 m, small `delta`, etc.,
   rather than the papers' values (L1 = 300 m, δ = 10 m). Tune in `config`.
3. **DR is a faithful surrogate.** Paper 2's exact recursive terminal-time /
   throughput computation (with terminal-speed coupling and the alternative
   energy formulations of §III.C) is approximated by an insert-and-evaluate
   search that minimizes the spread of nominal crossing times (earliest
   conflict-node arrival), with conflicting higher-priority CAVs pushing a CAV's
   crossing time by at least `headway`. The **priority ordering** it produces —
   which is what feeds the lateral CBFs — is the robust output; the absolute
   crossing-time numbers are approximate.
4. **Resequencing updates indices, not trajectories** (Paper 1 §V.E). This is the
   key efficiency of OCBF+DR and is implemented as such.
5. **The v2 wheelbase-`L` clobber bug is fixed here.** Because v3 is a fresh
   method with no result-preservation constraint, the arc-length variable is
   local (`arc_len`) and never overwrites the wheelbase `L`.
6. **Left-turn paths (3,6,9,12) use Euclidean distance** to conflict nodes (only
   right-turn paths 1,4,7,10 use arc length), matching the v2 convention. Refine
   if you need arc-accurate left-turn MP timing.
7. **Conflict-node arrival times** are estimated by rooting the *reference*
   position polynomial (the CAV may deviate from the reference under the CBF
   filter); adequate for sequencing and CBF timing, not an exact prediction.

## Parameters to tune (`config/simulation_config.m`)

- `beta` — time weight in the reference objective (larger = faster trajectories).
- `phi`, `delta` — safe gap `z >= phi*v + delta` (rear-end and lateral).
- `k_rear`, `k_lat` — class-K gains for the rear-end / lateral CBFs.
- `clf_weight`, `clf_epsilon`, `use_clf` — CLF speed-tracking relaxation weight,
  convergence rate, and on/off.
- `k_speed` — gain for the `vmax`/`vmin` barrier constraints.
- `headway` — minimum crossing-time separation used by the DR surrogate.
- `length_of_control_zone` (reference horizon Lk), `range_of_coordinator`.

The CBF/CLF gains in particular will need tuning in MATLAB for smooth,
collision-free behaviour at your chosen scale.

## Possible extensions (from the papers, not yet implemented)

- **HOCBF rear-end** for `phi = 0` (Paper 1 eqs. 28–29), to allow a purely
  distance-based rear-end constraint.
- **Feasibility-improved lateral constraint** (Paper 1 eq. 20) using the braking
  term when `v_i >= v_j`.
- **Noise / nonlinear dynamics** robustness (Paper 1 §IV.C, eqs. 21–25): add the
  threshold `c_k(t)` to violated constraints, or re-derive the Lie derivatives
  for the resistance-force model.
- **Paper 2's exact recursive terminal times** and alternative energy
  formulations, replacing the crossing-spread surrogate in `dynamic_resequencing`.

## Notes

- `data/coordinator.mat` and `data/_Last_Initial_Conditions.mat` are regenerated
  at runtime. If you want to load the fixed paper scenario
  `Initial_Conditions_for_paper.mat`, copy that binary into `data/` manually.
- The original `Intersection controller` folder and the reorganized
  `Intersection_Controller_v2` folder are untouched.
