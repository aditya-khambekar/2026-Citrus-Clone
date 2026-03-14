package org.team4639.frc2026.subsystems.pivot;


import edu.wpi.first.math.util.Units;

public class PivotConstants {
    public static final double MOTOR_TO_PIVOT_REDUCTION = 1.0 / 3.0 * 18.0 / 56.0 * 12.0 / 24.0;
    // figure out what this empirically
    public static final double MECHANISM_RANGE_ROTATIONS = Units.degreesToRotations(135);
    public static final double ZERO_AMPS = 12;
    public static final double ZERO_VOLTAGE = -1;

    public static final double IDLE_MECHANISM_ROTATIONS = 0;
    public static final double DOWN_MECHANISM_ROTATIONS = IDLE_MECHANISM_ROTATIONS + MECHANISM_RANGE_ROTATIONS;
}
