/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.commands.Actions;

public class AutoSegments {
    private static double INTAKE_EXTEND_DISTANCE_FROM_LINE = 0.75;
    public static Command LEFT_COPLF_FIRST_FAR(Actions actions) {
        return new ParallelDeadlineGroup(
                actions.followPath("COPLF_FIRST_FAR"),
                actions.idleSuperstructure(),
                new SequentialCommandGroup(
                        actions.pivotUp().until(() -> RobotState.getInstance().getEstimatedPose().getX() > FieldConstants.LinesVertical.allianceZone + INTAKE_EXTEND_DISTANCE_FROM_LINE),
                        new ParallelCommandGroup(actions.pivotDown(), actions.intake())
                )
        );
    }

    public static Command RIGHT_COPLF_FIRST_FAR(Actions actions) {
        return new ParallelDeadlineGroup(
                actions.followPathMirrored("COPLF_FIRST_FAR"),
                actions.idleSuperstructure(),
                new SequentialCommandGroup(
                        actions.pivotUp().until(() -> RobotState.getInstance().getEstimatedPose().getX() > FieldConstants.LinesVertical.allianceZone + INTAKE_EXTEND_DISTANCE_FROM_LINE),
                        new ParallelCommandGroup(actions.pivotDown(), actions.intake())
                )
        );
    }
}
