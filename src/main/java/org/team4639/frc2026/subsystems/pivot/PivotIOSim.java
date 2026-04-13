/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team4639.frc2026.Robot;

public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim pivotSim;
    private final PIDController pidController;
    private double appliedVolts = 0.0;

    public PivotIOSim() {
        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                1.0 / PivotConstants.MOTOR_TO_PIVOT_REDUCTION,
                0.003, // super fudged
                Units.inchesToMeters(12),
                Units.degreesToRadians(0),
                Units.degreesToRadians(135),
                true,
                Units.degreesToRadians(135),
                0.001, 0.0001
        );

        pidController = new PIDController(PivotConstants.SIM_kP, 0, 0);
    }

    @Override
    public void setVoltage(double volts) {
        this.appliedVolts = volts;
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void setSetpointEncoderRotations(double mechanismRotations) {
        this.appliedVolts = pidController.calculate(Units.radiansToRotations(pivotSim.getAngleRads()), mechanismRotations);
        pivotSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setPositionMechanismRotations(double mechanismRotations) {
        pivotSim.setState(Units.rotationsToRadians(mechanismRotations), 0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(Robot.defaultPeriodSecs);

        inputs.connected = true;
        inputs.celsius = 25;
        inputs.mechanismRotationsPerSecond = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());
        inputs.mechanismRotations = Units.radiansToRotations(pivotSim.getAngleRads());
        inputs.volts = appliedVolts;
        inputs.amps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {

    }
}
