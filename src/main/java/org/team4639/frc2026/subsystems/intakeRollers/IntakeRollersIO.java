/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double leftVolts;
    public double leftAmps;
    public double leftCelsius;
    public double leftMechanismRotationsPerSecond;
    public double leftMechanismRotations;

    public double rightVolts;
    public double rightAmps;
    public double rightCelsius;
    public double rightMechanismRotationsPerSecond;
    public double rightMechanismRotations;

    public boolean leftConnected;
    public boolean rightConnected;
  }

  default void setVoltage(double volts) {}

  default void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond) {}

  default void updateInputs(IntakeRollersIOInputs inputs) {}
}
