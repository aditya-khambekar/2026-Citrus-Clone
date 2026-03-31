/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class VerticalExtensionIOTalonFX implements VerticalExtensionIO {
  private final TalonFX verticalExtension;

  private final PositionVoltage positionVoltage;
  private final VoltageOut voltageOut;

  private final StatusSignal<Angle> rotorRotations;
  private final StatusSignal<AngularVelocity> rotorRotationsPerSecond;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> amps;
  private final StatusSignal<Temperature> celsius;

  public VerticalExtensionIOTalonFX(PortConfiguration ports) {
    verticalExtension = Phoenix6Factory.createDefaultTalon(ports.verticalExtension);

    PhoenixUtil.tryUntilOk(5, () -> verticalExtension.getConfigurator().apply(VerticalExtensionConfigs.verticalExtensionConfig));

    rotorRotations = verticalExtension.getRotorPosition();
    rotorRotationsPerSecond = verticalExtension.getRotorVelocity();
    volts = verticalExtension.getMotorVoltage();
    amps = verticalExtension.getStatorCurrent();
    celsius = verticalExtension.getDeviceTemp();

    voltageOut = new VoltageOut(0);
    positionVoltage = new PositionVoltage(0);
  }

  @Override
  public void setSetpointRotorRotations(double setpointRotorRotations) {
    positionVoltage.Position = Units.degreesToRotations(setpointRotorRotations);
    verticalExtension.setControl(positionVoltage);
    Logger.recordOutput("hopperExtension Controller Setpoint", positionVoltage.Position);
  }

  @Override
  public void updateInputs(VerticalExtensionIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                volts, amps,
                celsius, rotorRotationsPerSecond, rotorRotations)
            .isOK();
    inputs.volts = volts.getValueAsDouble();
    inputs.amps = amps.getValueAsDouble();
    inputs.celsius = celsius.getValueAsDouble();
    inputs.rotorRotations = rotorRotations.getValueAsDouble();
    inputs.rotorRotationsPerSecond = rotorRotationsPerSecond.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    verticalExtension.setVoltage(volts);
  }

  @Override
  public void setPositionRotorRotations(double rotorRotations) {
    verticalExtension.setPosition(rotorRotations);
  }

  @Override
  public void setBrakeMode(boolean isBrakeMode) {
    verticalExtension.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
