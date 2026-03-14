/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double leftVoltage;
    public double leftAmps;
    public double leftCurrent;
    public double leftTemperature;
    public double leftVelocity;

    public double rightVoltage;
    public double rightAmps;
    public double rightCurrent;
    public double rightTemperature;
    public double rightVelocity;

    public boolean connected = true;
  }

  default void setVoltage(double volts) {}

  default void setRotorVelocity(double velocity) {}

  default void stop() {}

  default void updateInputs(IntakeRollersIOInputs inputs) {}
}
