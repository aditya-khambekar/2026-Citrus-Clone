/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class Feeder extends FullSubsystem {
    private final RobotState state;
    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    @Getter
    private final FeederSysID sysID = new FeederSysID.FeederSysIDWPI(this, inputs);

    private record FeederSetpoint(double mechanismRotationsPerSecond, double mechanismRotationsPerSecondPerSecond) {
        static FeederSetpoint IDLE = new FeederSetpoint(0.0, 0.0);
    }

    public enum WantedState {
        IDLE,
        FEED_PASSING,
        FEED_SCORING
    }

    public enum SystemState {
        IDLE,
        FEED
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Feeder(FeederIO io, RobotState state) {
        this.io = io;
        this.state = state;

        Logger.recordOutput("Feeder/SystemState", systemState.toString());
        this.setDefaultCommand(this.run(this::runStateMachine));
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void periodicAfterScheduler() {

    }

    public FeederSetpoint getSetpoint() {
        return switch(wantedState) {
            case IDLE -> FeederSetpoint.IDLE;
            case FEED_PASSING -> {
                double currentDrumRPS = state.getPassingSetpoint(this).drumRotationsPerMinute() / 60.0;
                double nextDrumRPS = state.getNextPassingSetpoint(this).drumRotationsPerMinute() / 60.0;

                double drumRPSS = (nextDrumRPS - currentDrumRPS) / 0.02;

                yield new FeederSetpoint(currentDrumRPS * FeederConstants.FEED_PROPORTION_OF_DRUM, drumRPSS * FeederConstants.FEED_PROPORTION_OF_DRUM);
            }
            case FEED_SCORING -> {
                double currentDrumRPS = state.getScoringSetpoint(this).drumRotationsPerMinute() / 60.0;
                double nextDrumRPS = state.getNextScoringSetpoint(this).drumRotationsPerMinute() / 60.0;

                double drumRPSS = (nextDrumRPS - currentDrumRPS) / 0.02;

                yield new FeederSetpoint(currentDrumRPS * FeederConstants.FEED_PROPORTION_OF_DRUM, drumRPSS * FeederConstants.FEED_PROPORTION_OF_DRUM);
            }
        };
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Feeder/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
            case IDLE:
                handleIdle();
                break;
            case FEED:
                handleFeed();
                break;
        }
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case FEED_PASSING, FEED_SCORING -> SystemState.FEED;
        };
    }

    private void handleIdle() {
        io.setSetpointMechanismRotationsPerSecond(FeederConstants.IDLE_MECHANISM_RPM);
    }

    private void handleFeed() {
        var setpoint = getSetpoint();
        io.setSetpointMechanismRotationsPerSecond(setpoint.mechanismRotationsPerSecond, setpoint.mechanismRotationsPerSecondPerSecond);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    protected void setVoltage(Voltage volts){
        io.setVoltage(volts.in(Volts));
    }
}