package org.team4639.frc2026.subsystems.pivot;

import jdk.jfr.Percentage;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

public class Pivot extends FullSubsystem {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs;
    private final RobotState state;

    @Setter
    private double MANUAL_POSITION = PivotConstants.IDLE_MECHANISM_ROTATIONS;

    public Pivot(PivotIO io, RobotState state) {
        this.io = io;
        this.state = state;
        this.inputs = new PivotIOInputsAutoLogged();

        Logger.recordOutput("Pivot/SystemState", systemState);
        setDefaultCommand(this.run(this::runStateMachine));
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
        io.setPosition(PivotConstants.DOWN_MECHANISM_ROTATIONS);
    }

    private void handleIdle() {
        io.setPosition(PivotConstants.IDLE_MECHANISM_ROTATIONS);
    }

    private void handleManual() {
        io.setPosition(MANUAL_POSITION);
    }
}
