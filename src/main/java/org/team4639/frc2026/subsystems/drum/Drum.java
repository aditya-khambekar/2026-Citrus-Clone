/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drum;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

import java.util.Arrays;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Volts;
import static org.team4639.frc2026.subsystems.drum.DrumConstants.SHOOTING_RPM_TOLERANCE;

public class Drum extends FullSubsystem {
    private final RobotState state;
    private final DrumIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double PASSING_RPM = 0;
    private final double IDLE_VOLTAGE = 0;
    private double SCORING_RPM = 0;

    @Setter
    private double MANUAL_RPM = 0;

    @Getter
    private final DrumSysID sysID = new DrumSysID.DrumSysIDWPI(this, inputs);

    private record DrumSetpoint(double mechanismRotationsPerMinute, double mechanismRotationsPerMinutePerSecond){
        static DrumSetpoint IDLE = new DrumSetpoint(0.0, 0.0);
    }

    public enum WantedState {
        OFF,
        IDLE,
        SCORING,
        PASSING,
        MANUAL
    }

    public enum SystemState {
        OFF,
        IDLE,
        SCORING,
        PASSING,
        MANUAL
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public Drum(DrumIO io, RobotState state) {
        this.io = io;
        this.state = state;

        this.setDefaultCommand(this.run(this::runStateMachine));
        Logger.recordOutput("Drum/SystemState", systemState.toString());
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Drum", inputs);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void periodicAfterScheduler() {
        IntStream.range(0, inputs.connected.length).mapToObj(i -> inputs.connected[i]).forEach(state::acceptCANMeasurement);
        Arrays.stream(inputs.celsius).forEach(state::acceptTemperatureMeasurement);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case SCORING -> SystemState.SCORING;
            case PASSING -> SystemState.PASSING;
            case IDLE -> SystemState.IDLE;
            case MANUAL ->  SystemState.MANUAL;
            default -> SystemState.OFF;
        };
    }

    private void handleOff() {
        io.setVoltage(0);
    }

    private void handleScoring() {
        var setpoint = getSetpoint();
        io.setSetpointMechanismRPM(setpoint.mechanismRotationsPerMinute, setpoint.mechanismRotationsPerMinutePerSecond);
    }

    private void handlePassing() {
        var setpoint = getSetpoint();
        io.setSetpointMechanismRPM(setpoint.mechanismRotationsPerMinute, setpoint.mechanismRotationsPerMinutePerSecond);
    }

    private void handleIdle() {
        io.setVoltage(IDLE_VOLTAGE);
    }

    private void handleManual() {
        var setpoint = getSetpoint();
        io.setSetpointMechanismRPM(setpoint.mechanismRotationsPerMinute, setpoint.mechanismRotationsPerMinutePerSecond);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpoint().mechanismRotationsPerMinute, inputs.mechanismRPM[0], SHOOTING_RPM_TOLERANCE);
    }

    public boolean aboveSetpoint() {
        return inputs.mechanismRPM[0] + SHOOTING_RPM_TOLERANCE > getSetpoint().mechanismRotationsPerMinute;
    }

    /**
     * Should not be called in comp code. All usages of
     * setVoltage() needed for comp should be called internally.
     * @param volts voltage to set drum motors to
     */
    protected void setVoltage(Voltage volts) {
        io.setVoltage(volts.in(Volts));
    }

    public DrumSetpoint getSetpoint() {
        return switch(wantedState){
            case OFF, IDLE -> DrumSetpoint.IDLE;
            case SCORING -> {
                var desiredSetpoint = state.getScoringSetpoint(this);
                var nextSetpoint = state.getNextScoringSetpoint(this);
                var rotationsPerMinute = desiredSetpoint.drumRotationsPerMinute();
                var rotationsPerMinutePerSecond = (nextSetpoint.drumRotationsPerMinute() - rotationsPerMinute) / 0.02;

                yield new DrumSetpoint(rotationsPerMinute, rotationsPerMinutePerSecond);
            }
            case PASSING -> {
                var desiredSetpoint = state.getPassingSetpoint(this);
                var nextSetpoint = state.getNextPassingSetpoint(this);
                var rotationsPerMinute = desiredSetpoint.drumRotationsPerMinute();
                var rotationsPerMinutePerSecond = (nextSetpoint.drumRotationsPerMinute() - rotationsPerMinute) / 0.02;

                yield new DrumSetpoint(rotationsPerMinute, rotationsPerMinutePerSecond);
            }
            case MANUAL -> new DrumSetpoint(MANUAL_RPM, 0.0);
        };
    }


    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Drum/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.OFF;
        }

        switch (systemState) {
            case OFF:
                handleOff();
                break;
            case IDLE:
                handleIdle();
                break;
            case SCORING:
                handleScoring();
                break;
            case PASSING:
                handlePassing();
                break;
            case MANUAL:
                handleManual();
        }
    }
}