/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final double FULL_EXTENSION_ROTOR_ROTATIONS = 19.0 / 10 * 30 / 18 * 56 / 12;
  public static final double MOTOR_TO_HOOD_REDUCTION = 12.0 / 56.0 * 18.0 / 30.0 * 10.0 / 160.0;

  public static final double MIN_LAUNCH_DEGREES = 10;
  public static final double MAX_LAUNCH_DEGREES = MIN_LAUNCH_DEGREES + 360.0 * FULL_EXTENSION_ROTOR_ROTATIONS * MOTOR_TO_HOOD_REDUCTION;

  public static final double TOLERANCE_MECHANISM_ROTATIONS = Units.degreesToRotations(1);

  public static final double HOME_VOLTAGE = -3.0;
  public static final double HOME_CURRENT_THRESHOLD = 19.0;
}
