/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Volts;

public class VerticalExtension extends FullSubsystem {
    private final RobotState state;
    private final VerticalExtensionIO io;
    private final HopperExtensionIOInputsAutoLogged inputs = new HopperExtensionIOInputsAutoLogged();

    private double HOME_VOLTAGE = -3;

    private final double TOLERANCE_ROTOR_ROTATIONS = 0.5;

    public enum WantedState {
        IDLE,
        UP
    }

    public enum SystemState {
        HOME_DOWN,
        HOME_UP,
        IDLE,
        UP
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.HOME_DOWN;

    private WantedState lastZeroedWantedState = wantedState;

    public VerticalExtension(VerticalExtensionIO io, RobotState state) {
        this.io = io;
        this.state = state;

        this.setDefaultCommand(this.run(this::runStateMachine));

        Logger.recordOutput("hopperExtension/SystemState", systemState.toString());
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperExtension", inputs);
    }

    @Override
    public void periodic() {
        if (org.team4639.frc2026.Constants.tuningMode) {
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    io::applyNewGains,
                    PIDs.hopperExtensionKp,
                    PIDs.hopperExtensionKi,
                    PIDs.hopperExtensionKd,
                    PIDs.hopperExtensionKs,
                    PIDs.hopperExtensionKv,
                    PIDs.hopperExtensionKa,
                    PIDs.hopperExtensionKpSim,
                    PIDs.hopperExtensionKiSim,
                    PIDs.hopperExtensionKdSim);
        }

        if (this.systemState != SystemState.HOME_UP && this.systemState != SystemState.HOME_DOWN) {
            if (Math.abs(this.inputs.hopperExtensionCurrent) >= 12.0) {
                if (this.inputs.hopperExtensionVoltage < 0) {
                    io.setPositionRotorRotations(0);
                } else {
                    io.setPositionRotorRotations(Constants.FullExtensionRotorRotations);
                }
            }
        }
    }

    @Override
    public void periodicAfterScheduler() {
        //    state.setHopperExtensionStates(new Pair<>(wantedState, systemState));
        //    state.accept(inputs);
        //
        //    state.acceptCANMeasurement(inputs.hopperExtensionMotorConnected);
        //    state.acceptTemperatureMeasurement(inputs.pivotTemperature);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> {
                if (systemState == SystemState.HOME_DOWN) {
                    if (Math.abs(inputs.hopperExtensionCurrent) > 19.0) {
                        io.setPositionRotorRotations(0);
                        lastZeroedWantedState = WantedState.IDLE;
                        yield SystemState.IDLE;
                    } else {
                        yield SystemState.HOME_DOWN;
                    }
                }

                if (systemState == SystemState.HOME_UP) {
                    if (Math.abs(inputs.hopperExtensionCurrent) > 19.0) {
                        io.setPositionRotorRotations(Constants.FullExtensionRotorRotations);
                        lastZeroedWantedState = WantedState.IDLE;
                        yield SystemState.IDLE;
                    } else {
                        yield SystemState.HOME_UP;
                    }
                }

                yield SystemState.IDLE;
            }
            case UP -> SystemState.UP;
        };
    }

    private void handleHomeDown() {
        io.setVoltage(HOME_VOLTAGE);
        io.setBrakeMode(false);
    }

    private void handleHomeUp() {
        io.setVoltage(-HOME_VOLTAGE);
        io.setBrakeMode(true);
    }

    private void handleIdle() {
        io.setSetpointRotorRotations(0);
        io.setBrakeMode(false);
    }

    private void handleUp() {
        io.setSetpointRotorRotations(Constants.FullExtensionRotorRotations);
        io.setBrakeMode(true);
    }

    /**
     * Should not be called in comp code. All usages of setVoltage() needed for comp should be called
     * internally.
     *
     * @param volts
     */
    public void setVoltage(Voltage volts) {
        io.setVoltage(volts.in(Volts));
    }

    public double getSetpointRotorRotations() {
        return switch (systemState) {
            case IDLE, HOME_DOWN -> 0;
            case UP, HOME_UP -> Constants.FullExtensionRotorRotations;
        };
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(
                getSetpointRotorRotations(),
                inputs.hopperExtensionPositionDegrees,
                TOLERANCE_ROTOR_ROTATIONS);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("hopperExtension/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
            case HOME_DOWN:
                handleHomeDown();
                break;
            case HOME_UP:
                handleHomeUp();
                break;
            case IDLE:
                handleIdle();
                break;
            case UP:
                handleUp();
                break;
        }
    }
}
