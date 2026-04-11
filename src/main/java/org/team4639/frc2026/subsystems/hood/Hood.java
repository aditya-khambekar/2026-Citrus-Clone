/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class Hood extends FullSubsystem {
    private final RobotState state;
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final Debouncer homedDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);

    public enum WantedState {
        IDLE,
        SCORING,
        PASSING
    }

    public enum SystemState {
        HOME,
        IDLE,
        SCORING,
        PASSING
    }

    private record HoodSetpoint(double mechanismRotations, double mechanismRotationsPerSecond){
        static final HoodSetpoint IDLE = new HoodSetpoint(Units.degreesToRotations(HoodConstants.MIN_LAUNCH_DEGREES), 0.0);
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.HOME;

    public Hood(HoodIO io, RobotState state) {
        this.io = io;
        this.state = state;

        this.setDefaultCommand(this.run(this::runStateMachine));

        Logger.recordOutput("Hood/SystemState", systemState.toString());
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void periodicAfterScheduler() {

    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case PASSING -> SystemState.PASSING;
            case SCORING -> SystemState.SCORING;
            case IDLE -> {
                if (systemState == SystemState.HOME) {
                    if (homedDebouncer.calculate(Math.abs(inputs.amps) > HoodConstants.HOME_CURRENT_THRESHOLD)) {
                        io.setPositionMechanismRotations(Units.degreesToRotations(HoodConstants.MIN_LAUNCH_DEGREES));
                        yield SystemState.IDLE;
                    }
                    else yield SystemState.HOME;
                } else yield SystemState.IDLE;
            }
        };
    }

    public HoodSetpoint getSetpoint() {
        return switch (wantedState) {
            case IDLE -> HoodSetpoint.IDLE;
            case SCORING -> {
                 double setpointRotations = state.getScoringSetpoint(this).hoodRotations();
                 double nextSetpointRotations = state.getNextScoringSetpoint(this).hoodRotations();

                 double rotationsPerSecond = nextSetpointRotations - setpointRotations;
                 rotationsPerSecond /= 0.02;

                 yield new HoodSetpoint(setpointRotations, rotationsPerSecond);
            }
            case PASSING -> {
                double setpointRotations = state.getPassingSetpoint(this).hoodRotations();
                double nextSetpointRotations = state.getNextPassingSetpoint(this).hoodRotations();

                double rotationsPerSecond = nextSetpointRotations - setpointRotations;
                rotationsPerSecond /= 0.02;

                yield new HoodSetpoint(setpointRotations, rotationsPerSecond);
            }
        };
    }

    public void handleSetpointState(){
        HoodSetpoint setpoint = getSetpoint();
        io.setSetpointMechanismRotations(setpoint.mechanismRotations, setpoint.mechanismRotationsPerSecond);
    }

    public void handleHome() {
        io.setVoltage(HoodConstants.HOME_VOLTAGE);
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


    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Hood/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
            case IDLE, SCORING, PASSING -> handleSetpointState();
            case HOME -> handleHome();
        }
    }
}
