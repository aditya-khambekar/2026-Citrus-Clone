/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.ValueCacher;
import org.team4639.lib.util.FullSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class VerticalExtension extends FullSubsystem {
    private final RobotState state;
    private final VerticalExtensionIO io;
    private final VerticalExtensionIOInputsAutoLogged inputs = new VerticalExtensionIOInputsAutoLogged();

    public enum WantedState {
        IDLE,
        UP,
        MANUAL
    }

    public enum SystemState {
        HOME_UP,
        HOME_DOWN, // only done on startup, when there are not enough balls to fill hopper
        STUCK, // trying to go down, but too many balls
        IDLE,
        UP,
        MANUAL
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.HOME_DOWN;

    @Setter
    private double manualRotorRotations = VerticalExtensionConstants.MIN_ROTOR_ROTATIONS;

    private boolean beenHomedDown = false;
    private boolean beenHomedUp = false;

    private final Debouncer isStuckDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final ValueCacher<Object, Double> setpointCalculator = new ValueCacher<>(() -> {
        return switch (wantedState) {
            case IDLE -> VerticalExtensionConstants.MIN_ROTOR_ROTATIONS;
            case UP -> VerticalExtensionConstants.MAX_ROTOR_ROTATIONS;
            case MANUAL -> manualRotorRotations;
        };
    });

    public VerticalExtension(VerticalExtensionIO io, RobotState state) {
        this.io = io;
        this.state = state;

        this.setDefaultCommand(this.run(this::runStateMachine));

        Logger.recordOutput("VerticalExtension/SystemState", systemState.toString());
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("VerticalExtension", inputs);
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
        return switch (wantedState) {
            case IDLE -> {
                switch (systemState) {
                    case HOME_DOWN:
                        if (Math.abs(inputs.amps) > VerticalExtensionConstants.ZERO_CURRENT) {
                            this.beenHomedDown = true;
                            io.setPositionRotorRotations(VerticalExtensionConstants.MIN_ROTOR_ROTATIONS);
                            yield SystemState.IDLE;
                        } else yield SystemState.HOME_DOWN;
                    case IDLE:
                        if (!atSetpoint() && isStuckDebouncer.calculate(MathUtil.isNear(0, inputs.rotorRotationsPerSecond, 0.1))) {
                            yield SystemState.STUCK;
                        } else yield SystemState.IDLE;
                    case STUCK:
                        yield SystemState.STUCK; // only way to get unstuck is to change the wanted state, handled by code
                        default:
                            if (!beenHomedDown) yield SystemState.HOME_DOWN;
                            else yield SystemState.IDLE;
                }
            }
            case UP -> {
                if (systemState == SystemState.HOME_UP) {
                    if (Math.abs(inputs.amps) > VerticalExtensionConstants.ZERO_CURRENT) {
                        this.beenHomedUp = true;
                        io.setPositionRotorRotations(VerticalExtensionConstants.MAX_ROTOR_ROTATIONS);
                        yield SystemState.UP;
                    } else yield SystemState.HOME_UP;
                }
                if (!beenHomedUp) yield SystemState.HOME_UP;
                else yield SystemState.UP;
            }
            case MANUAL -> SystemState.MANUAL;
        };
    }

    public double getSetpointRotations() {
        return setpointCalculator.get(0b0);
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(setpointCalculator.get(0b1), inputs.rotorRotations, VerticalExtensionConstants.TOLERANCE_ROTOR_ROTATIONS);
    }

    private void handleIdle() {
        io.setSetpointRotorRotations(VerticalExtensionConstants.MIN_ROTOR_ROTATIONS);
    }

    private void handleUp() {
        io.setSetpointRotorRotations(VerticalExtensionConstants.MAX_ROTOR_ROTATIONS);
    }

    private void handleHomeUp() {
        io.setVoltage(VerticalExtensionConstants.ZERO_VOLTAGE);
    }

    private void handleHomeDown() {
        io.setVoltage(-VerticalExtensionConstants.ZERO_VOLTAGE);
    }

    private void handleStuck() {
        io.setVoltage(0);
    }

    private void handleManual() {
        io.setSetpointRotorRotations(manualRotorRotations);
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
            case UP:
                handleUp();
                break;
            case IDLE:
                handleIdle();
                break;
            case STUCK:
                handleStuck();
                break;
            case MANUAL:
                handleManual();
                break;
        }
    }
}
