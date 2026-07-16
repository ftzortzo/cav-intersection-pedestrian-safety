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

## Supplementary material for the paper Handling Uncertainty of Vulnerable Road Users in Coordinating Connected and Automated Vehicles at Signal-Free Intersections

On this website, we provide supplementary material to facilitate the understanding of our paper, entitled "Handling Uncertainty of Vulnerable Road Users in Coordinating Connected and Automated Vehicles at Signal-Free Intersections." The website is divided into several sections, each of which provides additional information corresponding to specific parts of the paper to further clarify the presented concepts. 

## Contributions

*From Section 1.3 of the paper:*

1. **Relaxed standstill assumption.** Expanding the approach of Sabouni et al. (2024) to allow CAVs to maintain a *nonzero* standstill distance $\tilde{\gamma}$ when arriving at the intersection from intersecting paths (Lemma 1, Theorem 1).
2. **VRU-aware controller.** Introducing a controller that accounts for VRU unpredictability at signal-free intersections operated by CAVs, via a HOCBF condition on the VRU unsafe set together with lane-boundary and steering-limit certificates (Propositions 3–5).
3. **Resequencing–replanning.** Incorporating an algorithm that safely re-plans the trajectories of all CAVs in the control zone when some deviate from their reference trajectories during the critical event of VRU presence (Section 5).
4. **Noise and higher-fidelity dynamics.** Extending the framework to account for measurement noise and vehicle dynamics with higher fidelity, including velocity-dependent longitudinal resistance and a friction-circle SOCP constraint (Section 6).

## Videos

*Recorded from the MATLAB / Simulink / RoadRunner co-simulation. Click any thumbnail to open on YouTube.*

<table>
<tr>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_1">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_1/hqdefault.jpg" alt="Scenario 1: baseline" width="100%"/>
</a>
<br/>
<b>Scenario 1 — Baseline</b><br/>
<sub>No VRU present. The framework reduces to Malikopoulos et al. (2021); throughput / delay reference for the pedestrian scenarios.</sub>
</td>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_2">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_2/hqdefault.jpg" alt="Scenario 2: one pedestrian" width="100%"/>
</a>
<br/>
<b>Scenario 2 — One pedestrian</b> <sub>(§7.1)</sub><br/>
<sub>Pedestrian crosses the south leg east-to-west. Seven vehicles affected; vehicle 1 stops for ~8.3 s and is demoted in the crossing sequence.</sub>
</td>
</tr>
<tr>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_3">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_3/hqdefault.jpg" alt="Scenario 3: two pedestrians" width="100%"/>
</a>
<br/>
<b>Scenario 3 — Two pedestrians</b> <sub>(§7.2)</sub><br/>
<sub>A second pedestrian crosses the east leg. Vehicles 4 and 7 both enter emergency mode; the resequencing mechanism propagates the delay through the conflict structure.</sub>
</td>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_4">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_4/hqdefault.jpg" alt="Vehicle 4 conflict of constraints" width="100%"/>
</a>
<br/>
<b>Vehicle 4 — Conflict of constraints</b> <sub>(§7.3)</sub><br/>
<sub>The VRU barrier and road-boundary conditions become simultaneously active. The QP avoids collision but transiently breaches the soft margin $r$.</sub>
</td>
</tr>
<tr>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_5">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_5/hqdefault.jpg" alt="Feasibility sweep" width="100%"/>
</a>
<br/>
<b>Feasibility sweep over $F_i(t_i^0)$</b> <sub>(§3.1)</sub><br/>
<sub>Iterating $t_i^f = t_i^f + \Delta t$ across the feasible interval until an unconstrained cubic trajectory satisfies (2), (3), and (5).</sub>
</td>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_6">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_6/hqdefault.jpg" alt="Real-time QP performance" width="100%"/>
</a>
<br/>
<b>Real-time QP performance</b> <sub>(§4.7)</sub><br/>
<sub>Per-step solve time for the emergency-mode QP. Constant in the number of CAVs; adds one affine constraint per detected VRU.</sub>
</td>
</tr>
<tr>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_7">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_7/hqdefault.jpg" alt="Scenario 2 with sensor noise" width="100%"/>
</a>
<br/>
<b>Scenario 2 with sensor noise</b> <sub>(§7.4)</sub><br/>
<sub>HDL-32E-consistent perception error; the inflated certificate ($r + \varepsilon_p$) preserves safety.</sub>
</td>
<td width="50%" valign="top">
<a href="https://www.youtube.com/watch?v=YOUTUBE_ID_8">
  <img src="https://img.youtube.com/vi/YOUTUBE_ID_8/hqdefault.jpg" alt="Scenario 3 with sensor noise" width="100%"/>
</a>
<br/>
<b>Scenario 3 with sensor noise</b> <sub>(§7.4)</sub><br/>
<sub>Same qualitative behavior as the noise-free run; no structural change to the QP.</sub>
</td>
</tr>
</table>

## Extended stress cases <sub>(§7.3)</sub>

Cases that drive the emergency-mode QP toward the boundary of feasibility — where the VRU barrier (29), road-boundary conditions (38), steering-angle limit (40), and friction-circle constraint (53) become simultaneously active. Each case is documented on the [companion site](https://ftzortzo.github.io/cav-intersection-pedestrian-safety/#stress) with configuration, constraint-interaction pattern, and observed margin.

## Alternative unsafe-set constructions <sub>(§4.2, Discussion 1)</sub>

The paper models each VRU's unsafe set as a disk

$$\mathcal{D} = \{(x,y) : (x - x_p)^2 + (y - y_p)^2 \le r^2\}$$

for clarity, but the framework is agnostic: any set for which we can write a candidate barrier function $b(\mathbf{x}) \ge 0$ works. The companion site carries out the analogous CBF derivation for three alternative representations from the literature:

- **Robust reachable set for a human agent** — Bajcsy et al. (2020)
- **Per-pedestrian reachability set** — Schratter et al. (2019)
- **Data-driven trajectory forecast** — Chen et al. (2023)

## Code layout

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

**Requirements.** MATLAB R2024a or later, Simulink, RoadRunner with the RoadRunner Scenario add-on.

**Quick start.** Setup and run instructions are documented per folder as the code is released. Start with `emergency_mode/` for the CBF-QP core, then `simulink_roadrunner/` for the full co-simulation.

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
