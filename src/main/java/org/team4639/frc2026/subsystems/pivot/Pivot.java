/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.Volts;

import javax.annotation.processing.Generated;

import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

import edu.wpi.first.units.measure.Voltage;

public class Pivot extends FullSubsystem {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs;
    private final RobotState state;

    @Getter
    private final PivotSysID sysID;

    @Setter
    private double manualMechanismRotations = PivotConstants.IDLE_MECHANISM_ROTATIONS;

    public Pivot(PivotIO io, RobotState state) {
        this.io = io;
        this.state = state;
        this.inputs = new PivotIOInputsAutoLogged();

        Logger.recordOutput("Pivot/SystemState", systemState);
        setDefaultCommand(this.run(this::runStateMachine));
        sysID = new PivotSysID.PivotSysIDWPI(this, inputs);
    }

    public enum WantedState {
        IDLE,
        DOWN,
        MANUAL
    }

    public enum SystemState {
        ZERO, // zero against up position hardstop
        IDLE,
        DOWN,
        MANUAL
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.ZERO;

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        state.setPivotMechanismRotations(inputs.mechanismRotations);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void periodicAfterScheduler() {
        state.acceptCANMeasurement(inputs.connected);
        state.acceptTemperatureMeasurement(inputs.celsius);
    }

    private SystemState handleStateTransitions() {
        return switch(wantedState) {
            case DOWN -> SystemState.DOWN;
            case IDLE -> {
                if (systemState == SystemState.ZERO) {
                    yield Math.abs(inputs.amps) > PivotConstants.ZERO_AMPS
                            ? SystemState.IDLE
                            : SystemState.ZERO;
                } else {
                    yield SystemState.IDLE;
                }
            }
            case MANUAL -> SystemState.MANUAL;
        };
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Pivot/SystemState", newState);
            systemState = newState;
        }

        switch (systemState) {
            case ZERO:
                handleZero();
                break;
            case DOWN:
                handleDown();
                break;
            case MANUAL:
                handleManual();
                break;
            case IDLE:
                handleIdle();
                break;
        }
    }

    private void handleZero() {
        io.setVoltage(PivotConstants.ZERO_VOLTAGE);
    }

    private void handleDown() {
        io.setSetpointMechanismRotations(PivotConstants.DOWN_MECHANISM_ROTATIONS);
    }

    private void handleIdle() {
        io.setSetpointMechanismRotations(PivotConstants.IDLE_MECHANISM_ROTATIONS);
    }

    private void handleManual() {
        io.setSetpointMechanismRotations(manualMechanismRotations);
    }

    protected void setVoltage(Voltage volts){
        io.setVoltage(volts.in(Volts));
    }
}
