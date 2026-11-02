# EV Battery Management System (BMS): MATLAB Simulation and Fault Validation

**Status:** Independent project starter / in progress  
**Implementation:** MATLAB script (base MATLAB functions; no specialist toolbox required)  
**Project type:** Simulation and software-level validation prototype; not a production BMS or vehicle test

## Project summary

This project models a simplified electric-vehicle battery pack and a small set of Battery Management System (BMS) monitoring functions. It is designed to demonstrate an engineering workflow that connects vehicle-load modelling, battery-state estimation, sensor monitoring, fault injection and repeatable validation checks.

The model uses an illustrative 96-series-cell, 60 Ah pack. A synthetic mixed urban/highway speed profile is converted into wheel and battery power using basic longitudinal vehicle dynamics. The pack model uses a representative open-circuit-voltage lookup curve, a lumped internal resistance and a first-order thermal model.

The BMS prototype includes:

- State-of-charge (SOC) propagation using measured pack current.
- A one-state Extended Kalman Filter (EKF) that combines current-based prediction with pack-voltage measurements.
- A first-order thermal observer for temperature-sensor consistency checks.
- Controlled voltage-sensor and temperature-sensor bias injection.
- Latched diagnostic flags and a simple voltage-measurement isolation fallback for SOC estimation.
- Automated model-level acceptance checks and CSV/PNG reporting.

## Validation scenarios

| Scenario | Injected condition | Expected diagnostic |
|---|---|---|
| Nominal baseline | Measurement noise only | No voltage or temperature fault flag |
| Voltage sensor bias | +8 V pack-measurement bias from 650 s to 800 s | Voltage sensor diagnostic latches; SOC estimator falls back to current-based propagation |
| Temperature sensor bias | +8 deg C temperature-measurement bias from 1250 s to 1400 s | Temperature sensor diagnostic latches |

The acceptance checks also enforce physical SOC bounds and an SOC estimation RMSE below 1.0 percentage point for these demonstration scenarios. The MATLAB script calculates the actual results at run time and writes them to the results CSV; no performance figures are assumed in advance.

## Run

1. Extract the project folder.
2. Open MATLAB and set the Current Folder to this directory.
3. Run:

```matlab
run_ev_bms_validation
```

The script creates a `results` directory containing:

- `validation_summary.csv`
- `bms_simulation_overview.png`
- `voltage_sensor_fault.png`
- `temperature_sensor_fault.png`

The script prints the test summary and raises an assertion if an acceptance check fails.

## Model assumptions and limitations

This is an educational simulation, not a validated electrochemical or vehicle model.

- The driving profile is synthetic and illustrative. It is **not** a measured vehicle trace, WLTP cycle, or certified test cycle.
- Battery voltage is based on a representative SOC-to-OCV lookup curve and a single equivalent pack resistance.
- Battery temperature uses a single-node lumped thermal model; cell-to-cell gradients, cooling-loop dynamics and detailed heat paths are not represented.
- The pack is represented by equivalent pack parameters. Individual cell imbalance, ageing, SOH estimation, balancing circuits and contactor dynamics are not modelled.
- Sensor faults are deliberately injected software signals, not faults generated on physical hardware.
- The diagnostic thresholds are demonstration values, not OEM limits or safety requirements.
- The work does not claim ISO 26262 compliance, AUTOSAR compliance, production ECU validation, HIL/SIL validation or vehicle-level certification.

## Suggested next steps

- Compare the synthetic profile with a properly sourced and documented standard drive-cycle dataset.
- Add cell-level voltage variation, balancing logic and parameter sensitivity tests.
- Add battery ageing and SOH estimation with a documented data source.
- Add a Simulink implementation and model-based test cases if Simulink is available.
- Define a CAN signal database and exchange BMS variables with a simulated ECU interface.
- Extend the test report with fault-detection delay, false-alarm rate and estimator robustness across parameter variations.

## Suggested CV wording while the work is unfinished

**EV Battery Management System (BMS) Modelling and Fault Validation | MATLAB | In progress**

- Developing a MATLAB simulation of an EV battery pack under an illustrative mixed-driving load profile, combining SOC estimation, battery thermal modelling and sensor monitoring.
- Implementing an EKF-based SOC estimator, controlled voltage/temperature sensor-fault injection and automated model-level validation checks.

Only describe the project as completed after running the script, reviewing the generated results and being able to explain the model assumptions and code.
