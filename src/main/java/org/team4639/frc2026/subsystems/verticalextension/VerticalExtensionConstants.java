/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.verticalextension;

public class VerticalExtensionConstants {
  public static final double ROTOR_RANGE_ROTATIONS = 8.0 / Math.PI * 56.0 / 18.0 * 9.0;
  public static final double UP_POSITION_ROTOR_ROTATIONS = 0;
  public static final double DOWN_POSITION_ROTOR_ROTATIONS = UP_POSITION_ROTOR_ROTATIONS - ROTOR_RANGE_ROTATIONS;

  public static final double ZERO_VOLTAGE = 5.0;
  public static final double ZERO_CURRENT = 12.0;
  public static final double STUCK_CURRENT = 12.0;

  public static final double TOLERANCE_ROTOR_ROTATIONS = 1.0;
}
