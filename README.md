# MAE547 MATLAB Robotics Toolbox

A MATLAB-based robotics simulation and analysis tool with a graphical interface. Define a
serial manipulator from its DH parameters, then compute and visualize its kinematics,
dynamics, and interaction control — all from a single GUI built by combining multiple
MATLAB scripts.

The modeling follows the formulations in Siciliano, Sciavicco, Villani & Oriolo,
*Robotics: Modelling, Planning and Control*.

## Features

Each is a button in the GUI:

1. **Forward kinematics** — end-effector transform and position from joint values.
2. **Inverse kinematics** — joint values to reach a desired pose, with convergence and error norm.
3. **Differential kinematics** — geometric (and analytical) Jacobian, mapping joint velocities to EE twist.
4. **Inverse velocity kinematics** — joint velocities for a desired twist via (damped) pseudo-inverse, with rank/conditioning diagnostics.
5. **Dynamics** — symbolic and numerical mass, Coriolis, and gravity terms; inverse and forward dynamics with time-history plots.
6. **Compliance control** — indirect force control against a one-sided virtual wall (PD + gravity compensation).
7. **Impedance control** — inverse-dynamics mass-spring-damper behavior with motion and force tracking.

Supports revolute (R) and prismatic (P) joints, configurable masses, link inertias, joint
limits, and environment parameters. Worked examples included for **RRR**, **RRP**,
**RRRP**, and **6R (RRRRRR)** manipulators.

## Getting started

```bash
git clone <your-repo-url>
```

Then in MATLAB:

1. Navigate to the cloned folder.
2. Open `robotics_toolbox_gui.m`.
3. Click **Run** (select *Add to Path* if prompted).
4. Enter the DH table, masses, and state/environment parameters, then **Build Robot**.

> Clone rather than downloading a zip — zipping can strip functionality the GUI relies on.

## Repository contents

- `robotics_toolbox_gui.m` and supporting MATLAB function scripts.
- `MAE547_Robotics_Toolkit_User_Manual.pdf` — full user manual: GUI walkthrough, each button explained, four worked example robots, and the underlying calculations (DH transforms, Jacobians, dynamics, contact-force model, compliance and impedance laws).

## Course

MAE 547 — Modeling and Control of Robots · Arizona State University

## Authors

Arvind Kaushik · Estrella Angel · Sabrina Farias · Portia Bryce · Kevin Teny Roy
