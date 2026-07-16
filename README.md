<h1 align="center">Handling Uncertainty of Vulnerable Road Users in Coordinating Connected and Automated Vehicles at Signal-Free Intersections</h1>

<p align="center">
  <a href="mailto:ft253@cornell.edu"><b>Filippos N. Tzortzoglou</b></a>
  &nbsp;·&nbsp;
  <a href="mailto:amaliko@cornell.edu"><b>Andreas A. Malikopoulos</b></a>
  <br/>
  <sub>Department of Civil and Environmental Engineering &nbsp;·&nbsp; Cornell University</sub>
  <br/>
  <sub>Preprint &nbsp;·&nbsp; <i>Automatica</i> &nbsp;·&nbsp; 2026</sub>
</p>

---

On this website, we provide supplementary material to facilitate the understanding of our paper, entitled *"Handling Uncertainty of Vulnerable Road Users in Coordinating Connected and Automated Vehicles at Signal-Free Intersections."* The material is presented in the same order it appears in the paper, starting with the low-level controller in §3.1, moving through the alternative unsafe-set representations in §4.2, the real-time feasibility of the emergency-mode QP in §4.7, the simulation sc:enarios in §7, and finishing with the code repository.

- [Identifying an unconstrained trajectory (§3.1)](#identifying-an-unconstrained-trajectory-31)
- [Alternative unsafe-set representations (§4.2)](#alternative-unsafe-set-representations-42)
- [Real-time feasibility of the emergency-mode QP (§4.7)](#real-time-feasibility-of-the-emergency-mode-qp-47)
- [Simulation scenarios (§7)](#simulation-scenarios-7)
- [Stress cases (§7.3)](#stress-cases-73)
- [Robustness to perception noise (§7.4)](#robustness-to-perception-noise-74)
- [Code repository (§7.5)](#code-repository-75)
- [Reproducing the simulations](#reproducing-the-simulations)
- [Citation](#citation)

## Identifying an unconstrained trajectory (§3.1)

In Section 3.1, we introduce the two-layer optimal control problem that yields an unconstrained trajectory for each CAV. When a CAV enters the control zone, we iteratively sweep the exit time $t_i^f$ across the feasible interval $F_i(t_i^0) = [\underline{t}_i^f, \overline{t}_i^f]$, checking at each step whether the resulting cubic trajectory satisfies all state, control, and safety constraints. The video below demonstrates this sweep in real time — even for very small values of $\Delta t$, the entire procedure completes within milliseconds.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_5">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_5/hqdefault.jpg" alt="Feasibility sweep over F_i(t_i^0)" width="720"/>
  </a>
</p>

## Alternative unsafe-set representations (§4.2)

Section 4.2 models each VRU's unsafe set as a disk

$$\mathcal{D} = \{(x,y) : (x - x_p)^2 + (y - y_p)^2 \le r^2\}$$

primarily for clarity of exposition. The framework itself is agnostic to that choice: any set for which we can write a candidate barrier function $b(\mathbf{x}) \ge 0$ can be plugged into the same construction. To make this concrete, we carry out the analogous CBF derivation — candidate barrier function, relative-degree check, and HOCBF condition — for three alternative representations from the literature:

- **Robust reachable set for a human agent** — Bajcsy et al. (2020)
- **Per-pedestrian reachability set** — Schratter et al. (2019)
- **Data-driven trajectory forecast** — Chen et al. (2023)

Each derivation is documented on the [companion site](https://ftzortzo.github.io/cav-intersection-pedestrian-safety/#alternative-sets).

## Real-time feasibility of the emergency-mode QP (§4.7)

Section 4.7 argues that the emergency-mode QP (44) is a small convex problem whose per-step solve time is essentially constant. All barrier conditions are affine in $(u_1, u_2)$, the decision variables are few, and the constraint count is bounded independent of the number of CAVs — each detected VRU adds only one affine constraint of the form (29). The following screencap records the solve time on our machine across a full simulation of Scenario 2, confirming that the per-step cost stays well within the $\Delta t = 0.02$&nbsp;s control step.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_6">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_6/hqdefault.jpg" alt="Per-step QP solve time" width="720"/>
  </a>
</p>

## Simulation scenarios (§7)

To validate the framework, Section 7 considers four scenarios at a four-leg intersection with a 100&nbsp;m control zone and a control step of $\Delta t = 0.02$&nbsp;s. Scenarios 1–3 share identical initial vehicle states, so any difference between them is attributable solely to the presence of the pedestrian(s).

### Baseline (no VRU)

The baseline is the reference for everything that follows: no VRU is present, so the framework reduces to that of Malikopoulos et al. (2021). It serves as the throughput and delay benchmark against which the pedestrian scenarios are compared.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_1">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_1/hqdefault.jpg" alt="Scenario 1: baseline" width="720"/>
  </a>
</p>

### Scenario 2 — single pedestrian (§7.1)

In Scenario 2, a pedestrian unexpectedly crosses the south leg of the intersection from east to west, interfering with seven vehicles. Vehicles 1 and 3 detect the pedestrian first and enter emergency mode; vehicle 1 eventually brakes to a full stop for roughly 8.3&nbsp;seconds, and the resequencing mechanism assigns it a low priority precisely because it is stopped — this prevents the stalled vehicle from obstructing others that are unaffected by the pedestrian. Once the pedestrian clears the road, vehicle 1 returns to its nominal path through recovery mode and is re-inserted into the crossing sequence.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_2">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_2/hqdefault.jpg" alt="Scenario 2: single pedestrian" width="720"/>
  </a>
</p>

### Scenario 3 — two pedestrians (§7.2)

Scenario 3 adds a second pedestrian on the east leg, moving north to south. The purpose is not to re-establish safety — that is already demonstrated in Scenario 2 — but to expose how the coordination layer responds to compounded disturbances. Vehicles 4 and 7 both enter emergency mode, and because they are conflicting vehicles of vehicle 1, the cascade keeps vehicle 1 at the lowest priority for 4.2&nbsp;seconds (compared with only 0.9&nbsp;s in Scenario 2). The additional pedestrian propagates through the conflict structure and lengthens the low-priority interval of an entirely different vehicle, illustrating that the resequencing mechanism reacts to the global state rather than to isolated interactions.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_3">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_3/hqdefault.jpg" alt="Scenario 3: two pedestrians" width="720"/>
  </a>
</p>

## Stress cases (§7.3)

Across Scenarios 2 and 3, vehicle 4 exhibits the smallest safety margin — 3.6 to 3.8&nbsp;m from the pedestrian, briefly breaching the $r = 4$&nbsp;m conservative disk though never entering the pedestrian's physical footprint. This happens because vehicle 4 approaches the south exit while the pedestrian is still on the lane, so the road-boundary barrier conditions and the VRU barrier condition become simultaneously active and compete for the available control authority. The video below shows this conflict of constraints in slow motion.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_4">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_4/hqdefault.jpg" alt="Vehicle 4 conflict of constraints" width="720"/>
  </a>
</p>

Beyond vehicle 4, we ran additional cases that drive the emergency-mode QP toward the boundary of feasibility, where the VRU barrier (29), road-boundary conditions (38), steering-angle limit (40), and friction-circle constraint (53) become simultaneously active. Each case, with its configuration, constraint-interaction pattern, and observed margin, is documented on the [companion site](https://ftzortzo.github.io/cav-intersection-pedestrian-safety/#stress).

## Robustness to perception noise (§7.4)

To assess robustness, Section 7.4 repeats Scenarios 2 and 3 with the estimated VRU position corrupted by bounded perception noise consistent with the Velodyne HDL-32E specification (see Appendix A). We enforce the certificate on an inflated safe set of radius $r + \varepsilon_p$ (Proposition 7), which preserves collision avoidance for any measurement consistent with the error bound — without any structural change to the QP. In both scenarios, the trajectories under noise are visually near-indistinguishable from the noise-free case.

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_7">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_7/hqdefault.jpg" alt="Scenario 2 with sensor noise" width="720"/>
  </a>
</p>

<p align="center">
  <a href="https://www.youtube.com/watch?v=YOUTUBE_ID_8">
    <img src="https://img.youtube.com/vi/YOUTUBE_ID_8/hqdefault.jpg" alt="Scenario 3 with sensor noise" width="720"/>
  </a>
</p>

## Code repository (§7.5)

The code that implements every part of the framework is organized to mirror the structure of the paper.

```
docs/                       Companion website (served via GitHub Pages)
low_level_pmp/              PMP-based unconstrained trajectory (§3.1)
cbf_qp_nominal/             Nominal CBF-QP with relaxed standstill (§3.2)
emergency_mode/             Bicycle model, HOCBFs, CLF, QP (§4)
resequencing/               ρ-factor priority and replanning (§5)
noise_and_dynamics/         Reinforced barriers, friction circle (§6)
simulink_roadrunner/        Full co-simulation with SimulinkVehicle_1/2
coordinator/                Coordinator kit generator
figures/                    Reproducibility for the paper's figures
```

## Reproducing the simulations

The full simulation environment requires MATLAB R2024a or later, Simulink, and RoadRunner with the RoadRunner Scenario add-on. Setup and run instructions are documented per folder as the code is released; for a first look at the emergency-mode controller in isolation, start with `emergency_mode/`, and then move to `simulink_roadrunner/` for the full co-simulation.

## Citation

```bibtex
@article{tzortzoglou2026vru,
  title   = {Handling Uncertainty of Vulnerable Road Users in Coordinating
             Connected and Automated Vehicles at Signal-Free Intersections},
  author  = {Tzortzoglou, Filippos N. and Malikopoulos, Andreas A.},
  journal = {Automatica},
  year    = {2026},
  note    = {Preprint}
}
```

## License

MIT — see [`LICENSE`](LICENSE).

## Contact

Correspondence: [ft253@cornell.edu](mailto:ft253@cornell.edu) &nbsp;·&nbsp; Information and Decision Science Laboratory, Cornell University.
