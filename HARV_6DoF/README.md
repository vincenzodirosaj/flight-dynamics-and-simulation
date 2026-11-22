# F/A-18 HARV — 6-DoF flight dynamics and constrained trim

This folder contains the MATLAB files used in the **6-DoF HARV investigation** described in Chapter 6 of the report, including aerodynamic functions, the constrained equilibrium search, quaternion-based integration and trajectory analysis. The separate Euler-angle manoeuvre from Chapter 3 is in `Additional_Maneuver/`.

## Running the main study

From the repository root:

```matlab
setup_paths
cd HARV_6DoF
Trim6DoF
```

The script searches for a candidate equilibrium with sideslip fixed at **2°**, then integrates the nonlinear 6-DoF equations over 100 seconds with fixed flight controls. It plots state histories and the simulated flight trajectory.

**Requirements:** MATLAB, Optimization Toolbox (`fmincon`) and Aerospace Toolbox, plus the helper scripts and data included in this folder.

To run the approximate spiral-mode investigation after a trim run:

```matlab
save('Trim.mat', 'x')
test_spiral
```

Simulation results and figures are documented in the [project report](../Report/Flight_Dynamics_and_Simulation_Di_Rosa_2023-2024.pdf).

## Numerical checks

The constrained search did not establish an exact trim state, and the resulting spiral-like trajectory should not be interpreted as proof of a positive spiral-mode eigenvalue. Before using freshly reproduced results, review two expressions in the original MATLAB code:

- In `costLongEquilibriumStaticStickFixed6DoF_Valphabetapqr.m`, the coefficient `1/myAC.mass*V` evaluates as `V/myAC.mass`; compare this to the `1/(myAC.mass*V)` factor in the lateral dynamics equation.
- In `Trim6DoF.m`, the expression for postprocessed flight-path angle `vGamma` uses trigonometric functions on angles already converted to degrees; MATLAB `sin` and `cos` expect radians.

The MATLAB source files implement the aircraft dynamics, aerodynamic models, and numerical simulations described in the report. See the [main README](../README.md) for the project overview and contributor credits.
