/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX leftLeader;
  private final TalonFX rightFollower;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltage;
  private final VoltageOut voltageOut;

  private final StatusSignal<Angle> leftMechanismRotations;
  private final StatusSignal<AngularVelocity> leftMechanismRotationsPerSecond;
  private final StatusSignal<Voltage> leftVolts;
  private final StatusSignal<Temperature> leftCelsius;
  private final StatusSignal<Current> leftAmps;

  private final StatusSignal<Angle> rightMechanismRotations;
  private final StatusSignal<AngularVelocity> rightMechanismRotationsPerSecond;
  private final StatusSignal<Voltage> rightVolts;
  private final StatusSignal<Temperature> rightCelsius;
  private final StatusSignal<Current> rightAmps;

  public IntakeRollersIOTalonFX(PortConfiguration portConfiguration) {
    this.leftLeader = Phoenix6Factory.createDefaultTalon(portConfiguration.leftIntake, false);
    this.rightFollower = Phoenix6Factory.createDefaultTalon(portConfiguration.rightIntake, false);

    PhoenixUtil.tryUntilOk(5, () -> leftLeader.getConfigurator().apply(IntakeRollersConfigs.leftConfig));
    PhoenixUtil.tryUntilOk(5, () -> rightFollower.getConfigurator().apply(IntakeRollersConfigs.rightConfig));

    rightFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    velocityVoltage = new VelocityVoltage(0);
    voltageOut = new VoltageOut(0);

    leftMechanismRotations = leftLeader.getPosition();
    leftMechanismRotationsPerSecond = leftLeader.getVelocity();
    leftVolts = leftLeader.getMotorVoltage();
    leftAmps = leftLeader.getTorqueCurrent();
    leftCelsius = leftLeader.getDeviceTemp();

    rightMechanismRotations = rightFollower.getPosition();
    rightMechanismRotationsPerSecond = rightFollower.getVelocity();
    rightVolts = rightFollower.getMotorVoltage();
    rightAmps = rightFollower.getTorqueCurrent();
    rightCelsius = rightFollower.getDeviceTemp();
  }

  @Override
  public void setVoltage(double volts) {
    leftLeader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond) {
    leftLeader.setControl(velocityVoltage.withVelocity(mechanismRotationsPerSecond));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.leftConnected =
        BaseStatusSignal.refreshAll(
                leftMechanismRotations,
            leftMechanismRotationsPerSecond,
            leftAmps,
            leftCelsius,
            leftVolts)
            .isOK();

    inputs.leftVolts = leftVolts.getValueAsDouble();
    inputs.leftAmps = leftAmps.getValueAsDouble();
    inputs.leftCelsius = leftCelsius.getValueAsDouble();
    inputs.leftMechanismRotationsPerSecond = leftMechanismRotationsPerSecond.getValueAsDouble();
    inputs.leftMechanismRotations = leftMechanismRotations.getValueAsDouble();

    inputs.rightConnected = BaseStatusSignal.refreshAll(
            rightMechanismRotations,
            rightMechanismRotationsPerSecond,
            rightAmps,
            rightCelsius,
            rightVolts
    ).isOK();

    inputs.rightVolts = rightVolts.getValueAsDouble();
    inputs.rightAmps = rightAmps.getValueAsDouble();
    inputs.rightCelsius = rightCelsius.getValueAsDouble();
    inputs.rightMechanismRotationsPerSecond = rightMechanismRotationsPerSecond.getValueAsDouble();
    inputs.rightMechanismRotations = rightMechanismRotations.getValueAsDouble();
  }
}
