/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

import org.littletonrobotics.junction.AutoLog;

public interface VerticalExtensionIO {
  default void setSetpointRotorRotations(double rotorRotations) {}

  default void setVoltage(double volts) {}

  default void updateInputs(VerticalExtensionIOInputs inputs) {}

  default void setPositionRotorRotations(double rotorRotations) {}

  default void setBrakeMode(boolean isBrakeMode) {}

  @AutoLog
  class VerticalExtensionIOInputs {
    public boolean connected = true;
    public double volts = 0.0;
    public double amps = 0.0;
    public double celsius = 0.0;
    public double rotorRotations = 0.0;
    public double rotorRotationsPerSecond = 0.0;
  }
}
