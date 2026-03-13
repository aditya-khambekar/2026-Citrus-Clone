/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopperextension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class HopperExtensionIOTalonFX implements HopperExtensionIO {
  private final TalonFX hopperExtensionMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final PositionVoltage request = new PositionVoltage(0);

  private final StatusSignal<Angle> hopperExtensionPosition;
  private final StatusSignal<AngularVelocity> hopperExtensionVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorCurrent;

  public HopperExtensionIOTalonFX(PortConfiguration ports) {
    hopperExtensionMotor = Phoenix6Factory.createDefaultTalon(ports.verticalExtension);

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // do NOT change this
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20;
    config.Audio.BeepOnConfig = false;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.ClosedLoopGeneral.ContinuousWrap = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = 0;
    applyNewGains();

    hopperExtensionPosition = hopperExtensionMotor.getPosition();
    hopperExtensionVelocity = hopperExtensionMotor.getVelocity();
    motorVoltage = hopperExtensionMotor.getMotorVoltage();
    motorCurrent = hopperExtensionMotor.getStatorCurrent();
  }

  @Override
  public void setSetpointRotorRotations(double setpointRotorRotations) {
    request.Position = Units.degreesToRotations(setpointRotorRotations);
    hopperExtensionMotor.setControl(request);
    Logger.recordOutput("hopperExtension Controller Setpoint", request.Position);
  }

  @Override
  public void updateInputs(HopperExtensionIOInputs inputs) {
    inputs.hopperExtensionMotorConnected =
        BaseStatusSignal.refreshAll(
                motorVoltage,
                motorCurrent,
                hopperExtensionMotor.getDeviceTemp(),
                hopperExtensionVelocity,
                hopperExtensionPosition)
            .isOK();
    inputs.hopperExtensionVoltage = motorVoltage.getValueAsDouble();
    inputs.hopperExtensionCurrent = motorCurrent.getValueAsDouble();
    inputs.hopperExtensionTemperature = hopperExtensionMotor.getDeviceTemp().getValueAsDouble();
    inputs.hopperExtensionPositionDegrees = hopperExtensionPosition.getValueAsDouble() * 360;
    inputs.hopperExtensionVelocityDegrees = hopperExtensionVelocity.getValueAsDouble() * 360;

    Logger.recordOutput("hopperExtension Rotations", hopperExtensionPosition.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    hopperExtensionMotor.setVoltage(volts);
  }

  public void updateGains() {
    config.Slot0.kP = PIDs.hopperExtensionKp.get();
    config.Slot0.kI = PIDs.hopperExtensionKi.get();
    config.Slot0.kD = PIDs.hopperExtensionKd.get();
    config.Slot0.kS = PIDs.hopperExtensionKs.get();
    config.Slot0.kV = PIDs.hopperExtensionKv.get();
    config.Slot0.kA = PIDs.hopperExtensionKa.get();
  }

  @Override
  public void applyNewGains() {
    updateGains();
    PhoenixUtil.tryUntilOk(5, () -> hopperExtensionMotor.getConfigurator().apply(config));
  }

  @Override
  public void setPositionRotorRotations(double rotorRotations) {
    hopperExtensionMotor.setPosition(rotorRotations);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    hopperExtensionMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
