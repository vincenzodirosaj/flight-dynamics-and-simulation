# Speed brakes — Simulink controlled-flight study

This folder contains the Simulink aircraft model, MATLAB setup and plotting scripts for the **speed-brake investigation** described in Chapter 2 of the project report.

## Included files

- `nape_speed_brakes.slx` — controlled-flight Simulink model with aerodynamic braking logic.
- `setup_speed_brakes.m` — aircraft properties, flight commands, control gains and speed-brake parameters.
- `plot_speed_brakes_results.m` — simulation-output postprocessing and figures.
- `myplotTrajectoryAndBody_Nape.m` and `aircraft.mat` — aircraft trajectory visualization resources.

## Running the model

From the repository root:

```matlab
setup_paths
cd Speed_Brakes
setup_speed_brakes
out = sim('nape_speed_brakes');
plot_speed_brakes_results
```

The project is configured for MATLAB R2024b with Simulink, Aerospace Toolbox and Aerospace Blockset.

## Model logic

The 500-second scenario updates the commanded flight state at **5, 90, 150 and 240 seconds**. When the thrust controller requests negative thrust, the physical thrust command is saturated at zero and a separate speed-brake branch adds drag and modifies the angle of attack needed to maintain commanded lift.

The speed-brake coefficients are set in `setup_speed_brakes.m`. **Parameter note:** the provided setup specifies `SB_DragFactorMax = 1.5` (up to +50% of baseline `CD0`), whereas Chapter 2 of the report describes an upper bound of +40%. Use the corresponding parameter configuration when comparing numerical results.

The [project report](../Report/Flight_Dynamics_and_Simulation_Di_Rosa_2023-2024.pdf) documents the model assumptions, simulated flight responses and comparisons. See the [main README](../README.md) for the broader project and contributor credits.
