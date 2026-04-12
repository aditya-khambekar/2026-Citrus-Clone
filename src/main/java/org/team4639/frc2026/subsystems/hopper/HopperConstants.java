/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopper;

import org.team4639.lib.util.LoggedTunableNumber;

public class HopperConstants {
    public static final double MOTOR_TO_ROLLER_REDUCTION = 12.0 / 26.0;

    public static final LoggedTunableNumber ON_MECHANISM_RPS = new LoggedTunableNumber("Hopper RPS", 30);
    public static final double IDLE_MECHANISM_RPS = 0;
}
