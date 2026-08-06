/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;


import edu.wpi.first.math.util.Units;

public class PivotConstants {
    public static final double MOTOR_TO_PIVOT_REDUCTION = 1.0 / 3.0 * 18.0 / 56.0 * 12.0 / 24.0;
    public static final double MOTOR_TO_ENCODER_REDUCTION = 1.0 / 3.0 * 18.0 / 56.0;
    public static final double ENCODER_TO_PIVOT_REDUCTION = 12.0 / 24.0;
    // figure out what this empirically
    public static final double MECHANISM_RANGE_ROTATIONS = Units.degreesToRotations(135);
    public static final double ZERO_AMPS = 12;
    public static final double ZERO_VOLTAGE = -1;

    public static final double IDLE_MECHANISM_ROTATIONS = -0.3;

    public static final double DOWN_MECHANISM_ROTATIONS = 0.46;
    public static final double UP_MECHANISM_ROTATIONS = -0.35;

    // Volts per MECHANISM rotation
    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0;

    // Volts per MECHANISM rotation
    public static final double SIM_kP = 3;
    public static final double SIM_kI = 0;
    public static final double SIM_kD = 0;
}
