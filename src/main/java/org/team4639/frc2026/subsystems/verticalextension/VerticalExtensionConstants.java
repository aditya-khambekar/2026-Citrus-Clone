/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

public class VerticalExtensionConstants {
  //TODO: figure out what this actually is
  public static final double ROTOR_RANGE_ROTATIONS = 8.0 / Math.PI * 56.0 / 18.0 * 9.0;
  public static final double MIN_ROTOR_ROTATIONS = 0;
  public static final double MAX_ROTOR_ROTATIONS = MIN_ROTOR_ROTATIONS + ROTOR_RANGE_ROTATIONS;

  public static final double ZERO_VOLTAGE = 5.0;
  public static final double ZERO_CURRENT = 12.0;
  public static final double STUCK_CURRENT = 12.0;

  public static final double TOLERANCE_ROTOR_ROTATIONS = 1.0;

  // minimum vertical extension proportion at which the intake has a full range of motion
  //TODO: figure out what this actually is
  public static final double INTAKE_SAFE_MIN_PROPORTION = 0.2;

  public static double rotorRotationsToProportion(double rotorRotations) {
    return (rotorRotations - MIN_ROTOR_ROTATIONS) / (MAX_ROTOR_ROTATIONS - MIN_ROTOR_ROTATIONS);
  }

  public static double proportionToRotorRotations(double proportion) {
    return MIN_ROTOR_ROTATIONS + proportion * (MAX_ROTOR_ROTATIONS - MIN_ROTOR_ROTATIONS);
  }
}
