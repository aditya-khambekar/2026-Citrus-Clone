/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import org.team4639.lib.util.LoggedTunableNumber;

public class FeederConstants {
    public static final double MOTOR_TO_FEEDER_REDUCTION = 12.0 / 30.0;

    public static final double IDLE_MECHANISM_RPM = 0;
    public static final LoggedTunableNumber FEED_SPEED = new LoggedTunableNumber("Feed Speed", 20); // greater than 1 since the diameters are different, we want the surface speed of this to be less
}
