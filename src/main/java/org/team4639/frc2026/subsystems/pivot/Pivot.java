/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.Volts;

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

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WantedState {
        IDLE,
        DOWN,
        MANUAL
    }

    public enum SystemState {
        IDLE,
        DOWN,
        MANUAL
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        state.setPivotMechanismRotations(inputs.mechanismRotations);
        state.isPivotUp = inputs.encoderRotations < (PivotConstants.UP_ENCODER_POSITION + PivotConstants.DOWN_ENCODER_POSITION) / 2.0;
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
            case IDLE -> SystemState.IDLE;
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

    private void handleDown() {
        io.setSetpointEncoderRotations(PivotConstants.DOWN_ENCODER_POSITION);
    }

    private void handleIdle() {
        if (inputs.encoderRotations < PivotConstants.UP_ENCODER_POSITION) io.setVoltage(0);
        else io.setSetpointEncoderRotations(PivotConstants.UP_ENCODER_POSITION);
    }

    private void handleManual() {
        io.setSetpointEncoderRotations(manualMechanismRotations);
    }

    protected void setVoltage(Voltage volts){
        io.setVoltage(volts.in(Volts));
    }
}
