/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage request = new VelocityVoltage(0);

  public IntakeRollersIOTalonFX(PortConfiguration portConfiguration) {
    this.leftMotor = Phoenix6Factory.createDefaultTalon(portConfiguration.leftIntake, false);
    this.rightMotor = Phoenix6Factory.createDefaultTalon(portConfiguration.rightIntake, false);

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;

    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));

    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRotorVelocity(double velocity) {
    leftMotor.setControl(request.withVelocity(velocity));
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {

    inputs.connected =
        BaseStatusSignal.refreshAll(
                leftMotor.getMotorVoltage(),
                leftMotor.getStatorCurrent(),
                leftMotor.getSupplyCurrent(),
                leftMotor.getDeviceTemp(),
                leftMotor.getVelocity(),
                rightMotor.getMotorVoltage(),
                rightMotor.getStatorCurrent(),
                rightMotor.getSupplyCurrent(),
                rightMotor.getDeviceTemp(),
                rightMotor.getVelocity())
            .isOK();

    inputs.leftVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemperature = leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftVelocity = leftMotor.getVelocity().getValueAsDouble();

    inputs.rightVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightTemperature = rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightVelocity = rightMotor.getVelocity().getValueAsDouble();
  }
}
