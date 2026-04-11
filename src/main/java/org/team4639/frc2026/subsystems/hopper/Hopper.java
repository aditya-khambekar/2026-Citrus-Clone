/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class Hopper extends FullSubsystem {
    private final RobotState state;
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private double unjamStartTime = Double.NaN;
    private final double unjamTimePeriod = 0.2;

    @Getter
    private final HopperSysID sysID = new HopperSysID.HopperSysIDWPI(this, inputs);

    public enum WantedState {
        IDLE,
        ON
    }

    public enum SystemState {
        IDLE,
        ON,
        UNJAM
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Hopper(HopperIO io, RobotState state) {
        this.io = io;
        this.state = state;

        Logger.recordOutput("Hopper/SystemState", systemState.toString());
        this.setDefaultCommand(this.run(this::runStateMachine));
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void periodicAfterScheduler() {
        state.acceptCANMeasurement(inputs.connected);
        state.acceptTemperatureMeasurement(inputs.celsius);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Hopper/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
            case IDLE:
                handleIdle();
                break;
            case ON:
                handleOn();
                break;
            case UNJAM:
                handleUnjam();
                break;
        }
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case ON -> {
                switch(systemState){
                    case IDLE -> {
                        yield SystemState.ON;
                    }
                    case ON -> {
                        if (false){
                            unjamStartTime = Timer.getTimestamp();
                            yield SystemState.UNJAM;
                        } else {
                            yield SystemState.ON;
                        }
                    }
                    case UNJAM -> {
                        if (Timer.getTimestamp() - unjamStartTime >= unjamTimePeriod){
                            yield SystemState.ON;
                        } else {
                            yield SystemState.IDLE;
                        }
                    }
                }
                yield SystemState.ON;

            }
        };
    }

    private void handleIdle() {
        io.setSetpointMechanismRotationsPerSecond(HopperConstants.IDLE_MECHANISM_RPS);
    }

    private void handleOn() {
        io.setSetpointMechanismRotationsPerSecond(HopperConstants.ON_MECHANISM_RPS.get());
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    protected void setVoltage(Voltage volts){
        io.setVoltage(volts.in(Volts));
    }

    private void handleUnjam() {
        io.setSetpointMechanismRotationsPerSecond(-HopperConstants.ON_MECHANISM_RPS.get());
    }
}