package org.team4639.frc2026.subsystems.intakeRollers;

import org.team4639.lib.util.LoggedTunableNumber;

public class IntakeRollersConstants {
    public static final double MOTOR_TO_ROLLER_REDUCTION = 12.0 / 26.0;

    public static final LoggedTunableNumber INTAKE_MECHANISM_RPS = new LoggedTunableNumber("Intake RPS", 35);
}
