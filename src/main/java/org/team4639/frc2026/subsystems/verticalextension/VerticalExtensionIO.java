/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

import org.littletonrobotics.junction.AutoLog;

public interface VerticalExtensionIO {
  default void setSetpointRotorRotations(double rotorRotations) {}

  default void setVoltage(double volts) {}

  default void updateInputs(HopperExtensionIOInputs inputs) {}

  default void applyNewGains() {}

  default void setPositionRotorRotations(double rotorRotations) {}

  default void setBrakeMode(boolean brake) {}

  @AutoLog
  class HopperExtensionIOInputs {
    public boolean hopperExtensionMotorConnected = true;
    public double hopperExtensionVoltage = 0.0;
    public double hopperExtensionCurrent = 0.0;
    public double hopperExtensionTemperature = 0.0;
    public double hopperExtensionPositionDegrees = 0.0;
    public double hopperExtensionVelocityDegrees = 0.0;
  }
}
