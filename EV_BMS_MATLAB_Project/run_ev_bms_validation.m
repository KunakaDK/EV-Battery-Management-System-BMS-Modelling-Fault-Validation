function run_ev_bms_validation()
% RUN_EV_BMS_VALIDATION
% MATLAB-only simulation of a simplified EV battery pack and BMS monitor.
% No specialised MATLAB toolboxes or external datasets are required.
%
% Run from MATLAB with:
%   run_ev_bms_validation
%
% The script creates a results folder containing a CSV test summary and
% PNG plots for the nominal and sensor-fault scenarios.

clc;
close all;
rng(42, 'twister');

cfg.dt_s = 1;
cfg.duration_s = 1800;
cfg.seriesCells = 96;
cfg.capacity_Ah = 60;
cfg.packResistance_ohm = 0.075;
cfg.thermalCapacity_J_per_K = 220e3;
cfg.thermalConductance_W_per_K = 25;
cfg.ambientTemp_C = 25;
cfg.initialSOC = 0.86;
cfg.initialSOCError = -0.02;
cfg.currentNoiseStd_A = 0.25;
cfg.voltageNoiseStd_V = 0.60;
cfg.temperatureNoiseStd_C = 0.20;
cfg.voltageResidualThreshold_V = 6.0;
cfg.temperatureResidualThreshold_C = 4.5;
cfg.ekfProcessNoise = 1e-7;
cfg.ekfVoltageNoiseVariance = 1.0^2;
cfg.regenPowerLimit_W = 35e3;
cfg.auxiliaryPower_W = 700;

cases(1) = struct('name', 'Nominal baseline', 'fault', 'none');
cases(2) = struct('name', 'Voltage sensor bias', 'fault', 'voltage');
cases(3) = struct('name', 'Temperature sensor bias', 'fault', 'temperature');

results = simulate_bms_case(cfg, cases(1).fault);
for k = 2:numel(cases)
    rng(42, 'twister'); % identical noise sequence for fair scenario comparison
    results(k) = simulate_bms_case(cfg, cases(k).fault);
end

summary = build_summary(cases, results);
projectDir = fileparts(mfilename('fullpath'));
resultsDir = fullfile(projectDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
writetable(summary, fullfile(resultsDir, 'validation_summary.csv'));

plot_case_overview(results(2), cfg, resultsDir);
plot_fault_residual(results(2).time_s, results(2).voltageResidual_V, ...
    results(2).voltageFaultFlag, results(2).voltageFaultInjected, ...
    cfg.voltageResidualThreshold_V, 'Voltage sensor diagnostic', 'Residual (V)', ...
    fullfile(resultsDir, 'voltage_sensor_fault.png'));
plot_fault_residual(results(3).time_s, results(3).temperatureResidual_C, ...
    results(3).temperatureFaultFlag, results(3).temperatureFaultInjected, ...
    cfg.temperatureResidualThreshold_C, 'Temperature sensor diagnostic', 'Residual (deg C)', ...
    fullfile(resultsDir, 'temperature_sensor_fault.png'));

fprintf('\nEV BMS simulation and validation summary\n');
disp(summary);
fprintf('Results saved in: %s\n', resultsDir);

% Automated acceptance checks. These are model-level checks, not automotive
% certification or proof of production ECU behaviour.
assert(all(results(1).socTrue >= 0 & results(1).socTrue <= 1), ...
    'Nominal SOC left its physical bounds.');
assert(all(results(1).socEstimate >= 0 & results(1).socEstimate <= 1), ...
    'Nominal SOC estimate left its physical bounds.');
assert(results(1).socRmse_pctPoints < 1.0, ...
    'Nominal SOC estimation RMSE exceeded the project acceptance threshold.');
assert(~any(results(1).voltageFaultFlag) && ~any(results(1).temperatureFaultFlag), ...
    'A diagnostic alarm was raised in the nominal scenario.');
assert(any(results(2).voltageFaultFlag & results(2).voltageFaultInjected), ...
    'The injected voltage sensor fault was not detected.');
assert(~any(results(2).temperatureFaultFlag), ...
    'The voltage-fault scenario raised an unrelated temperature alarm.');
assert(any(results(3).temperatureFaultFlag & results(3).temperatureFaultInjected), ...
    'The injected temperature sensor fault was not detected.');
assert(~any(results(3).voltageFaultFlag), ...
    'The temperature-fault scenario raised an unrelated voltage alarm.');

fprintf('All model-level acceptance checks passed.\n');
end

function out = simulate_bms_case(cfg, faultType)
% Build a synthetic mixed urban/highway speed profile. This is not a WLTP
% or other certified drive cycle.
time_s = (0:cfg.dt_s:cfg.duration_s)';
n = numel(time_s);
[speed_mps, wheelPower_W] = make_drive_profile(time_s, cfg);

socTrue = zeros(n, 1);
temperatureTrue_C = zeros(n, 1);
packOCV_V = zeros(n, 1);
packCurrent_A = zeros(n, 1);
packTerminalVoltage_V = zeros(n, 1);
socTrue(1) = cfg.initialSOC;
temperatureTrue_C(1) = cfg.ambientTemp_C;

for k = 1:n
    packOCV_V(k) = cfg.seriesCells * cell_ocv(socTrue(k));
    batteryPower_W = wheelPower_W(k);
    discriminant = max(packOCV_V(k)^2 - 4 * cfg.packResistance_ohm * batteryPower_W, 1);
    current = (packOCV_V(k) - sqrt(discriminant)) / (2 * cfg.packResistance_ohm);
    packCurrent_A(k) = min(max(current, -150), 220);
    packTerminalVoltage_V(k) = packOCV_V(k) - packCurrent_A(k) * cfg.packResistance_ohm;

    if k < n
        socTrue(k + 1) = clamp_value(socTrue(k) - ...
            packCurrent_A(k) * cfg.dt_s / (cfg.capacity_Ah * 3600), 0.01, 0.99);
        heat_W = packCurrent_A(k)^2 * cfg.packResistance_ohm;
        cooling_W = cfg.thermalConductance_W_per_K * ...
            (temperatureTrue_C(k) - cfg.ambientTemp_C);
        temperatureTrue_C(k + 1) = temperatureTrue_C(k) + ...
            cfg.dt_s / cfg.thermalCapacity_J_per_K * (heat_W - cooling_W);
    end
end

% Add realistic measurement noise and one controlled sensor fault per case.
currentMeasured_A = packCurrent_A + cfg.currentNoiseStd_A * randn(n, 1);
voltageMeasured_V = packTerminalVoltage_V + cfg.voltageNoiseStd_V * randn(n, 1);
temperatureMeasured_C = temperatureTrue_C + cfg.temperatureNoiseStd_C * randn(n, 1);
voltageFaultInjected = false(n, 1);
temperatureFaultInjected = false(n, 1);

if strcmp(faultType, 'voltage')
    voltageFaultInjected = time_s >= 650 & time_s <= 800;
    voltageMeasured_V(voltageFaultInjected) = voltageMeasured_V(voltageFaultInjected) + 8.0;
elseif strcmp(faultType, 'temperature')
    temperatureFaultInjected = time_s >= 1250 & time_s <= 1400;
    temperatureMeasured_C(temperatureFaultInjected) = ...
        temperatureMeasured_C(temperatureFaultInjected) + 8.0;
end

% One-state Extended Kalman Filter for SOC. The voltage measurement is
% isolated after a voltage-sensor fault is latched; SOC then coasts on the
% current-based prediction for the remainder of the run.
socEstimate = zeros(n, 1);
voltageResidual_V = zeros(n, 1);
voltageFaultFlag = false(n, 1);
P = 0.02^2;
socEstimate(1) = clamp_value(cfg.initialSOC + cfg.initialSOCError, 0.01, 0.99);
voltageFaultLatched = false;

% Independent first-order thermal observer for temperature-sensor checks.
temperatureResidual_C = zeros(n, 1);
temperatureFaultFlag = false(n, 1);
temperatureModel_C = cfg.ambientTemp_C;
temperatureFaultLatched = false;

for k = 1:n
    if k > 1
        socPredicted = clamp_value(socEstimate(k - 1) - ...
            currentMeasured_A(k - 1) * cfg.dt_s / (cfg.capacity_Ah * 3600), 0.01, 0.99);
        Ppredicted = P + cfg.ekfProcessNoise;
    else
        socPredicted = socEstimate(1);
        Ppredicted = P;
    end

    voltagePredicted_V = cfg.seriesCells * cell_ocv(socPredicted) - ...
        currentMeasured_A(k) * cfg.packResistance_ohm;
    voltageResidual_V(k) = voltageMeasured_V(k) - voltagePredicted_V;
    if abs(voltageResidual_V(k)) > cfg.voltageResidualThreshold_V
        voltageFaultLatched = true;
    end

    temperatureResidual_C(k) = temperatureMeasured_C(k) - temperatureModel_C;
    if abs(temperatureResidual_C(k)) > cfg.temperatureResidualThreshold_C
        temperatureFaultLatched = true;
    end

    voltageFaultFlag(k) = voltageFaultLatched;
    temperatureFaultFlag(k) = temperatureFaultLatched;

    if voltageFaultLatched
        socEstimate(k) = socPredicted;
        P = Ppredicted;
    else
        deltaSOC = 1e-3;
        H = cfg.seriesCells * (cell_ocv(socPredicted + deltaSOC) - ...
            cell_ocv(socPredicted - deltaSOC)) / (2 * deltaSOC);
        kalmanGain = Ppredicted * H / ...
            (H^2 * Ppredicted + cfg.ekfVoltageNoiseVariance);
        socEstimate(k) = clamp_value(socPredicted + ...
            kalmanGain * voltageResidual_V(k), 0.01, 0.99);
        P = max((1 - kalmanGain * H) * Ppredicted, 1e-10);
    end

    if k < n
        heatEstimate_W = currentMeasured_A(k)^2 * cfg.packResistance_ohm;
        coolingEstimate_W = cfg.thermalConductance_W_per_K * ...
            (temperatureModel_C - cfg.ambientTemp_C);
        temperatureModel_C = temperatureModel_C + ...
            cfg.dt_s / cfg.thermalCapacity_J_per_K * ...
            (heatEstimate_W - coolingEstimate_W);
    end
end

out.time_s = time_s;
out.speed_mps = speed_mps;
out.wheelPower_W = wheelPower_W;
out.socTrue = socTrue;
out.socEstimate = socEstimate;
out.temperatureTrue_C = temperatureTrue_C;
out.temperatureMeasured_C = temperatureMeasured_C;
out.packCurrent_A = packCurrent_A;
out.voltageMeasured_V = voltageMeasured_V;
out.packTerminalVoltage_V = packTerminalVoltage_V;
out.voltageResidual_V = voltageResidual_V;
out.temperatureResidual_C = temperatureResidual_C;
out.voltageFaultFlag = voltageFaultFlag;
out.temperatureFaultFlag = temperatureFaultFlag;
out.voltageFaultInjected = voltageFaultInjected;
out.temperatureFaultInjected = temperatureFaultInjected;
out.socRmse_pctPoints = sqrt(mean((socEstimate - socTrue).^2)) * 100;
out.peakTemperature_C = max(temperatureTrue_C);
out.faultType = faultType;
end

function [speed_mps, batteryPower_W] = make_drive_profile(time_s, cfg)
% Waypoints define a repeatable illustrative urban/suburban/highway profile.
waypointTime_s = [0 20 40 70 100 130 160 200 240 280 320 360 ...
    420 480 540 600 660 720 780 840 900 960 1020 1080 ...
    1140 1200 1260 1320 1380 1440 1500 1560 1620 1680 1740 1800];
waypointSpeed_kmh = [0 0 30 50 35 0 0 40 65 30 0 0 ...
    50 70 70 80 100 105 95 110 110 100 80 60 ...
    70 80 50 35 0 0 40 60 50 25 0 0];
speed_mps = interp1(waypointTime_s, waypointSpeed_kmh, time_s, 'linear') / 3.6;
acceleration_mps2 = gradient(speed_mps, cfg.dt_s);

vehicleMass_kg = 1800;
rollingResistance = 0.012;
airDensity_kg_m3 = 1.225;
dragArea_m2 = 0.62;
drivetrainEfficiency = 0.91;
regenerativeEfficiency = 0.65;
gravity_mps2 = 9.81;

rollingForce_N = vehicleMass_kg * gravity_mps2 * rollingResistance;
aeroForce_N = 0.5 * airDensity_kg_m3 * dragArea_m2 .* speed_mps.^2;
tractionForce_N = vehicleMass_kg .* acceleration_mps2 + rollingForce_N + aeroForce_N;
wheelPower_W = tractionForce_N .* speed_mps;
batteryPower_W = zeros(size(wheelPower_W));
traction = wheelPower_W >= 0;
batteryPower_W(traction) = wheelPower_W(traction) / drivetrainEfficiency + cfg.auxiliaryPower_W;
batteryPower_W(~traction) = wheelPower_W(~traction) * regenerativeEfficiency + cfg.auxiliaryPower_W;
batteryPower_W = max(batteryPower_W, -cfg.regenPowerLimit_W);
end

function voltage_V = cell_ocv(soc)
% Representative cell open-circuit-voltage lookup curve, not cell-specific data.
socBreakpoints = [0.00 0.05 0.10 0.20 0.50 0.80 0.90 0.95 1.00];
voltageBreakpoints_V = [3.00 3.35 3.48 3.60 3.72 3.92 4.05 4.14 4.20];
voltage_V = interp1(socBreakpoints, voltageBreakpoints_V, ...
    clamp_value(soc, 0, 1), 'linear');
end

function value = clamp_value(value, lowerBound, upperBound)
value = min(max(value, lowerBound), upperBound);
end

function summary = build_summary(cases, results)
count = numel(cases);
caseName = cell(count, 1);
injectedFault = cell(count, 1);
socRmse_pctPoints = zeros(count, 1);
peakTemperature_C = zeros(count, 1);
detectionLatency_s = NaN(count, 1);
voltageFaultDetected = false(count, 1);
temperatureFaultDetected = false(count, 1);
acceptancePassed = false(count, 1);

for k = 1:count
    caseName{k} = cases(k).name;
    injectedFault{k} = cases(k).fault;
    socRmse_pctPoints(k) = results(k).socRmse_pctPoints;
    peakTemperature_C(k) = results(k).peakTemperature_C;
    voltageFaultDetected(k) = any(results(k).voltageFaultFlag & results(k).voltageFaultInjected);
    temperatureFaultDetected(k) = any(results(k).temperatureFaultFlag & results(k).temperatureFaultInjected);

    if strcmp(cases(k).fault, 'voltage')
        idx = find(results(k).voltageFaultFlag & results(k).voltageFaultInjected, 1, 'first');
        if ~isempty(idx)
            detectionLatency_s(k) = results(k).time_s(idx) - 650;
        end
        acceptancePassed(k) = voltageFaultDetected(k) && ...
            ~any(results(k).temperatureFaultFlag) && socRmse_pctPoints(k) < 1.0;
    elseif strcmp(cases(k).fault, 'temperature')
        idx = find(results(k).temperatureFaultFlag & results(k).temperatureFaultInjected, 1, 'first');
        if ~isempty(idx)
            detectionLatency_s(k) = results(k).time_s(idx) - 1250;
        end
        acceptancePassed(k) = temperatureFaultDetected(k) && ...
            ~any(results(k).voltageFaultFlag) && socRmse_pctPoints(k) < 1.0;
    else
        acceptancePassed(k) = ~any(results(k).voltageFaultFlag) && ...
            ~any(results(k).temperatureFaultFlag) && socRmse_pctPoints(k) < 1.0;
    end
end

summary = table(caseName, injectedFault, socRmse_pctPoints, peakTemperature_C, ...
    detectionLatency_s, voltageFaultDetected, temperatureFaultDetected, acceptancePassed, ...
    'VariableNames', {'Scenario', 'InjectedFault', 'SOCRMSE_pctPoints', ...
    'PeakPackTemp_degC', 'DetectionLatency_s', 'VoltageFaultDetected', ...
    'TemperatureFaultDetected', 'AcceptancePassed'});
end

function plot_case_overview(result, cfg, resultsDir)
fig = figure('Color', 'w', 'Position', [100 100 1050 800]);
subplot(3, 1, 1);
plot(result.time_s, result.speed_mps * 3.6, 'LineWidth', 1.1);
grid on;
ylabel('Speed (km/h)');
title('EV battery-management simulation - illustrative drive profile');

subplot(3, 1, 2);
plot(result.time_s, result.socTrue * 100, 'LineWidth', 1.2);
hold on;
plot(result.time_s, result.socEstimate * 100, '--', 'LineWidth', 1.1);
grid on;
ylabel('SOC (%)');
legend('Model SOC', 'EKF estimate', 'Location', 'best');

subplot(3, 1, 3);
plot(result.time_s, result.temperatureTrue_C, 'LineWidth', 1.2);
hold on;
plot(result.time_s, result.temperatureMeasured_C, ':', 'LineWidth', 0.9);
grid on;
xlabel('Time (s)');
ylabel('Temperature (deg C)');
legend('Model temperature', 'Measured temperature', 'Location', 'best');

saveas(fig, fullfile(resultsDir, 'bms_simulation_overview.png'));
close(fig);
end

function plot_fault_residual(time_s, residual, faultFlag, injectedFault, threshold, ...
    plotTitle, yLabelText, outputFile)
fig = figure('Color', 'w', 'Position', [120 120 1050 520]);
plot(time_s, residual, 'LineWidth', 1.0);
hold on;
plot(time_s, threshold * ones(size(time_s)), '--', 'LineWidth', 1.0);
plot(time_s, -threshold * ones(size(time_s)), '--', 'LineWidth', 1.0);
plot(time_s, double(faultFlag) * threshold * 0.85, 'LineWidth', 1.0);
plot(time_s, double(injectedFault) * threshold * 0.65, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel(yLabelText);
title(plotTitle);
legend('Measurement-model residual', 'Positive threshold', 'Negative threshold', ...
    'Latched diagnostic flag (scaled)', 'Injected fault window (scaled)', ...
    'Location', 'best');
saveas(fig, outputFile);
close(fig);
end
