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
  <a href="https://youtu.be/x330OcMrvow">
    <img src="https://img.youtube.com/vi/x330OcMrvow/hqdefault.jpg" alt="Feasibility sweep over F_i(t_i^0)" width="720"/>
  </a>
</p>

## Alternative unsafe-set representations (§4.2)

## Ellipsoidal unsafe set — CBF derivation

Section 4.2 of the paper models each VRU's unsafe set as a disk

$$\mathcal{D} = \{(x,y) : (x - x_p)^2 + (y - y_p)^2 \le r^2\}$$

primarily for clarity of exposition. The framework itself is agnostic to that choice: any set for which we can write a candidate barrier function $b(\mathbf{x}_b) \ge 0$ can be plugged into the same construction. In this note, we carry out the analogous derivation for an **ellipsoidal** unsafe set, whose semi-axes can be tuned to encode physical structure that a disk cannot — for instance, extending the set along the pedestrian's walking direction (accounting for their expected motion), shortening it perpendicular to that direction (where the pedestrian is less likely to move), or growing it with the vehicle's approach speed.

We follow exactly the four steps of Proposition 3 in the paper: (i) define the candidate barrier function, (ii) verify its relative degree with respect to the dynamics, (iii) compute the second-order Lie derivatives, and (iv) assemble the HOCBF condition. As a sanity check, we then show that setting the two semi-axes equal recovers the paper's disk condition (29) exactly.

### Setup

Recall the control-affine bicycle model (20)–(21) of the paper:

$$\dot{x} = v\cos\theta, \quad \dot{y} = v\sin\theta, \quad \dot{\theta} = \frac{v}{\sigma}\,u_1, \quad \dot{v} = u_2,$$

with state $\mathbf{x}_b = [x, y, \theta, v]^T$, drift field $f(\mathbf{x}_b) = [v\cos\theta, v\sin\theta, 0, 0]^T$, and control vector fields $g_1(\mathbf{x}_b) = [0, 0, v/\sigma, 0]^T$, $g_2(\mathbf{x}_b) = [0, 0, 0, 1]^T$.

### Axis-aligned ellipse

Let the pedestrian be located at $(x_p, y_p)$, and let $a, b > 0$ be the semi-axes of the elliptical unsafe set. Consider the axis-aligned ellipse

$$\mathcal{E} = \left\{(x, y) : \frac{(x - x_p)^2}{a^2} + \frac{(y - y_p)^2}{b^2} \le 1\right\},$$

and define the candidate barrier function

$$b_{\text{ell}}(\mathbf{x}_b) = \frac{(x - x_p)^2}{a^2} + \frac{(y - y_p)^2}{b^2} - 1.$$

The associated safe set is the zero-superlevel set

$$\mathcal{S}_{\text{ell}} = \{\mathbf{x}_b \in \mathbb{R}^4 : b_{\text{ell}}(\mathbf{x}_b) \ge 0\},$$

which is the exterior of $\mathcal{E}$.

**Relative degree.** The gradient of $b_{\text{ell}}$ with respect to $\mathbf{x}_b$ is

$$\nabla b_{\text{ell}}(\mathbf{x}_b) = \left[\frac{2(x - x_p)}{a^2},\ \frac{2(y - y_p)}{b^2},\ 0,\ 0\right].$$

The first-order Lie derivatives are

$$L_f b_{\text{ell}} = \nabla b_{\text{ell}} \cdot f = \frac{2v(x - x_p)\cos\theta}{a^2} + \frac{2v(y - y_p)\sin\theta}{b^2},$$

$$L_{g_1} b_{\text{ell}} = 0, \qquad L_{g_2} b_{\text{ell}} = 0,$$

confirming that neither control input appears after a single differentiation. Hence $b_{\text{ell}}$ has relative degree two with respect to the bicycle dynamics, and — as in the paper's disk case — the second-order HOCBF framework of §4.3 must be invoked.

**Second-order Lie derivatives.** Taking the gradient of $L_f b_{\text{ell}}$,

$$\nabla L_f b_{\text{ell}} =
\begin{bmatrix}
\dfrac{2v\cos\theta}{a^2} \\[4pt]
\dfrac{2v\sin\theta}{b^2} \\[6pt]
2v\left[-\dfrac{(x-x_p)\sin\theta}{a^2} + \dfrac{(y-y_p)\cos\theta}{b^2}\right] \\[10pt]
\dfrac{2(x-x_p)\cos\theta}{a^2} + \dfrac{2(y-y_p)\sin\theta}{b^2}
\end{bmatrix}^T .$$

Combining with $f$, $g_1$, and $g_2$ yields

$$L_f^2 b_{\text{ell}} = \nabla L_f b_{\text{ell}} \cdot f = 2v^2\left(\frac{\cos^2\theta}{a^2} + \frac{\sin^2\theta}{b^2}\right),$$

$$L_{g_1} L_f b_{\text{ell}} = \nabla L_f b_{\text{ell}} \cdot g_1 = \frac{2v^2}{\sigma}\left[-\frac{(x-x_p)\sin\theta}{a^2} + \frac{(y-y_p)\cos\theta}{b^2}\right],$$

$$L_{g_2} L_f b_{\text{ell}} = \nabla L_f b_{\text{ell}} \cdot g_2 = \frac{2(x-x_p)\cos\theta}{a^2} + \frac{2(y-y_p)\sin\theta}{b^2}.$$

**HOCBF condition.** Substituting into the second-order barrier condition (28) with class-$\mathcal{K}$ functions chosen as the identity,

$$L_f^2 b_{\text{ell}} + L_{g_1} L_f b_{\text{ell}}\, u_1 + L_{g_2} L_f b_{\text{ell}}\, u_2 + 2 L_f b_{\text{ell}} + b_{\text{ell}} \ge 0,$$

gives the explicit barrier condition for the axis-aligned ellipse:

$$
\begin{aligned}
& 2v^2\left(\frac{\cos^2\theta}{a^2} + \frac{\sin^2\theta}{b^2}\right)
+ \frac{2v^2}{\sigma}\left[-\frac{(x-x_p)\sin\theta}{a^2} + \frac{(y-y_p)\cos\theta}{b^2}\right] u_1 \\[6pt]
& + 2\left[\frac{(x-x_p)\cos\theta}{a^2} + \frac{(y-y_p)\sin\theta}{b^2}\right] u_2
+ 4v\left[\frac{(x-x_p)\cos\theta}{a^2} + \frac{(y-y_p)\sin\theta}{b^2}\right] \\[6pt]
& + \frac{(x-x_p)^2}{a^2} + \frac{(y-y_p)^2}{b^2} - 1 \ \ge \ 0.
\end{aligned}
$$

The inequality is affine in $(u_1, u_2)$, and therefore can be embedded as a single half-space constraint in the emergency-mode QP (44), exactly as the disk condition (29).

**Sanity check — reduction to the disk.** Setting $a = b = r$, every occurrence of $\cos^2\theta / a^2 + \sin^2\theta / b^2$ collapses to $1/r^2$, and every bracketed expression collapses to its unweighted disk analogue. Multiplying the condition through by $r^2$, we recover

$$
\begin{aligned}
& 2v^2 + \frac{2v^2}{\sigma}\left[-(x-x_p)\sin\theta + (y-y_p)\cos\theta\right] u_1 \\
& + 2\left[(x-x_p)\cos\theta + (y-y_p)\sin\theta\right] u_2 \\
& + 4v\left[(x-x_p)\cos\theta + (y-y_p)\sin\theta\right] \\
& + (x-x_p)^2 + (y-y_p)^2 - r^2 \ \ge \ 0,
\end{aligned}
$$

which is exactly the paper's HOCBF condition (29).

### Rotated ellipse

To align the ellipse with the pedestrian's walking direction, let $\phi$ denote the orientation angle of the major axis with respect to the global $x$-axis, and define the rotated coordinates

$$\tilde{x} = (x - x_p)\cos\phi + (y - y_p)\sin\phi, \qquad \tilde{y} = -(x - x_p)\sin\phi + (y - y_p)\cos\phi.$$

The rotated ellipse is

$$\mathcal{E}_\phi = \left\{(x,y) : \frac{\tilde{x}^2}{a^2} + \frac{\tilde{y}^2}{b^2} \le 1\right\}.$$

Introducing the displacement vector $\mathbf{d} = [x - x_p,\ y - y_p]^T$ and the symmetric positive-definite matrix

$$P = R(\phi)^T \begin{pmatrix} 1/a^2 & 0 \\ 0 & 1/b^2 \end{pmatrix} R(\phi), \qquad R(\phi) = \begin{pmatrix} \cos\phi & \sin\phi \\ -\sin\phi & \cos\phi \end{pmatrix},$$

the candidate barrier function takes the compact quadratic form

$$b_{\text{ell},\phi}(\mathbf{x}_b) = \mathbf{d}^T P\, \mathbf{d} - 1.$$

Writing $\mathbf{e}_\theta = [\cos\theta, \sin\theta]^T$ for the heading unit vector and $\mathbf{e}_\theta^\perp = [-\sin\theta, \cos\theta]^T$, the derivation proceeds exactly as before. The first-order Lie derivative is

$$L_f b_{\text{ell},\phi} = 2v\, \mathbf{d}^T P\, \mathbf{e}_\theta,$$

and $L_{g_1} b_{\text{ell},\phi} = L_{g_2} b_{\text{ell},\phi} = 0$, so the relative degree is again two. The second-order Lie derivatives are

$$L_f^2 b_{\text{ell},\phi} = 2v^2\, \mathbf{e}_\theta^T P\, \mathbf{e}_\theta,$$

$$L_{g_1} L_f b_{\text{ell},\phi} = \frac{2v^2}{\sigma}\, \mathbf{d}^T P\, \mathbf{e}_\theta^\perp, \qquad L_{g_2} L_f b_{\text{ell},\phi} = 2\, \mathbf{d}^T P\, \mathbf{e}_\theta,$$

and the HOCBF condition becomes

$$2v^2\, \mathbf{e}_\theta^T P\, \mathbf{e}_\theta \ +\ \frac{2v^2}{\sigma}\, \mathbf{d}^T P\, \mathbf{e}_\theta^\perp\, u_1 \ +\ 2\, \mathbf{d}^T P\, \mathbf{e}_\theta\, u_2 \ +\ 4v\, \mathbf{d}^T P\, \mathbf{e}_\theta \ +\ \mathbf{d}^T P\, \mathbf{d} - 1 \ \ge \ 0.$$

Setting $P = (1/r^2)\, I$ recovers the paper's disk condition (29) after multiplying through by $r^2$; setting $\phi = 0$ recovers the axis-aligned expression above.

### Parametrizing the semi-axes

Because the barrier condition is affine in $(u_1, u_2)$ regardless of the numerical values of $a$, $b$, and $\phi$, we can update these parameters at each control step without changing the structure of the QP. This makes the ellipse a natural vehicle for encoding physical structure that a disk cannot represent. Three parametrizations are worth highlighting.

**Orientation from the pedestrian's walking direction.** Let $\phi_p$ denote the pedestrian's heading estimated from consecutive detections. Setting $\phi = \phi_p$ orients the ellipse's major axis along the walking direction, which extends the unsafe region in the direction the pedestrian is expected to move and contracts it laterally — encoding, in the geometry itself, the fact that pedestrians rarely change direction abruptly.

**Semi-axes from the pedestrian's speed.** Let $v_p$ denote the pedestrian's speed. A common choice is

$$a = a_0 + \kappa_a v_p, \qquad b = b_0,$$

with $a_0, b_0, \kappa_a > 0$. This grows the ellipse along the walking direction in proportion to how fast the pedestrian is moving, and matches the "reaction reachable set" intuition used by Bajcsy et&nbsp;al. (2020) and Schratter et&nbsp;al. (2019). The lateral semi-axis $b_0$ is kept fixed and typically small, encoding the vehicle footprint plus a lateral safety margin.

**Semi-axes from the vehicle's approach speed.** Alternatively (or additionally), the semi-axes can scale with the vehicle's own speed $v$ to reflect its longer stopping distance at higher speeds:

$$a = a_0 + \kappa_v v, \qquad b = b_0 + \kappa_v' v.$$

Since $v$ is a state variable, this makes $b_{\text{ell},\phi}$ explicitly $v$-dependent, and the Lie derivative expressions above must be augmented with terms of the form $\partial_v b_{\text{ell},\phi}$. In practice, however, we treat the semi-axes as parameters held constant across a single control step $\Delta t$: this preserves the derivation as stated and updates $a, b$ from the measured speed at each step. The resulting closed-loop behavior is that the ellipse "breathes" with speed while the QP remains a small, static-parameter convex problem, which is important for real-time feasibility (§4.7).

Analogous constructions for reachability-based and data-driven trajectory-forecast unsafe sets follow the same pattern: define $b(\mathbf{x}_b) \ge 0$ so that its zero-superlevel set is the exterior of the desired unsafe region, verify the relative degree, and assemble the HOCBF condition.


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
