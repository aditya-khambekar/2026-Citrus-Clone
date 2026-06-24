/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.json.simple.parser.ParseException;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.commands.factory.DriveCommands;
import org.team4639.frc2026.commands.factory.SuperstructureCommands;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.drum.Drum;
import org.team4639.frc2026.subsystems.feeder.Feeder;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.hopper.Hopper;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollers;
import org.team4639.frc2026.subsystems.pivot.Pivot;

import java.io.IOException;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class AutoCommands {

    public static final double EXTEND_INTAKE_METERS_PAST_LINE = 0.5;

    public static Command OP_LEFT(
            Drive drive,
            Drum drum,
            Hood hood,
            Hopper hopper,
            Feeder feeder,
            Pivot pivot,
            IntakeRollers intake,
            RobotState state) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        followPath("OPL-0", true, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(SuperstructureCommands.stop(intake), SuperstructureCommands.retract(pivot))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake))),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper)),
                new ParallelDeadlineGroup(
                        followPath("OPL-1", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                        .until(() -> state.getEstimatedPose().getX()
                                                < FieldConstants.LinesVertical.allianceZone),
                                setRotationOverride(drive, state),
                                Commands.waitSeconds(0.5),
                                SuperstructureCommands.score(drum, hood, feeder, hopper)
                                        .alongWith(SuperstructureCommands.agitate(pivot)),
                                SuperstructureCommands.intake(intake))),
                new ParallelDeadlineGroup(
                        followPath("OPL-2", false, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper),
                        new SequentialCommandGroup(
                                SuperstructureCommands.stop(intake)
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPath("OPL-3", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper),
                        SuperstructureCommands.intake(intake)),
                new ParallelCommandGroup(
                        DriveCommands.autoScoringAlign(drive),
                        SuperstructureCommands.agitate(pivot),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )));
    }

    public static Command OP_RIGHT(
            Drive drive,
            Drum drum,
            Hood hood,
            Hopper hopper,
            Feeder feeder,
            Pivot pivot,
            IntakeRollers intake,
            RobotState state) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-0", true, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(SuperstructureCommands.stop(intake), SuperstructureCommands.retract(pivot))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake))),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper)),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-1", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                        .until(() -> state.getEstimatedPose().getX()
                                                < FieldConstants.LinesVertical.allianceZone),
                                setRotationOverride(drive, state),
                                Commands.waitSeconds(0.5),
                                SuperstructureCommands.score(drum, hood, feeder, hopper)
                                        .alongWith(SuperstructureCommands.agitate(pivot)),
                                SuperstructureCommands.intake(intake))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-2", false, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper),
                        new SequentialCommandGroup(
                                SuperstructureCommands.stop(intake)
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-3", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper),
                        SuperstructureCommands.intake(intake)),
                new ParallelCommandGroup(
                        DriveCommands.autoScoringAlign(drive),
                        SuperstructureCommands.agitate(pivot),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )));
    }

    public static Command OP_NEAR_LEFT(
            Drive drive,
            Drum drum,
            Hood hood,
            Hopper hopper,
            Feeder feeder,
            Pivot pivot,
            IntakeRollers intake,
            RobotState state) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        followPath("OPNL-0", true, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(SuperstructureCommands.stop(intake), SuperstructureCommands.retract(pivot))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake))),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper)),
                new ParallelDeadlineGroup(
                        followPath("OPL-1", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                        .until(() -> state.getEstimatedPose().getX()
                                                < FieldConstants.LinesVertical.allianceZone),
                                setRotationOverride(drive, state),
                                Commands.waitSeconds(0.5),
                                SuperstructureCommands.score(drum, hood, feeder, hopper)
                                        .alongWith(SuperstructureCommands.agitate(pivot)),
                                SuperstructureCommands.intake(intake))),
                new ParallelDeadlineGroup(
                        followPath("OPL-2", false, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper),
                        new SequentialCommandGroup(
                                SuperstructureCommands.stop(intake)
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPath("OPL-3", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper),
                        SuperstructureCommands.intake(intake)),
                new ParallelCommandGroup(
                        DriveCommands.autoScoringAlign(drive),
                        SuperstructureCommands.agitate(pivot),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )));
    }

    public static Command OP_NEAR_RIGHT(
            Drive drive,
            Drum drum,
            Hood hood,
            Hopper hopper,
            Feeder feeder,
            Pivot pivot,
            IntakeRollers intake,
            RobotState state) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        followPathMirrored("OPNL-0", true, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(SuperstructureCommands.stop(intake), SuperstructureCommands.retract(pivot))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake))),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper)),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-1", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                        .until(() -> state.getEstimatedPose().getX()
                                                < FieldConstants.LinesVertical.allianceZone),
                                setRotationOverride(drive, state),
                                Commands.waitSeconds(0.5),
                                SuperstructureCommands.score(drum, hood, feeder, hopper)
                                        .alongWith(SuperstructureCommands.agitate(pivot)),
                                SuperstructureCommands.intake(intake))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-2", false, true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(drum, hood, feeder, hopper),
                        new SequentialCommandGroup(
                                SuperstructureCommands.stop(intake)
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        SuperstructureCommands.extend(pivot), SuperstructureCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-3", false, true, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper),
                        SuperstructureCommands.intake(intake)),
                new ParallelCommandGroup(
                        DriveCommands.autoScoringAlign(drive),
                        SuperstructureCommands.agitate(pivot),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )));
    }

    /**
     * Follow PathPlanner path. If this is the start of an auto routine, then reset
     * pose to the starting pose of the path. Otherwise, pathfind to the start of
     * the path.
     *
     * @param pathName  name of the Path in PathPlanner
     * @param resetPose whether to reset the pose to the path's starting pose.
     */
    private static Command followPath(String pathName, boolean resetPose, boolean resetOverrides, RobotState state) {

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Command resetOverridesCommand = Commands.runOnce(() -> {
                if (resetOverrides) PPHolonomicDriveController.setRotationTargetOverride(null);
            });
            Command beforeStarting = Commands.none();
            if (resetPose) {
                beforeStarting = Commands.runOnce(
                        () -> state.resetPose(path.getStartingHolonomicPose().get()));
            }
            var lastTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();

            if (resetPose)
                return beforeStarting
                        .andThen(resetOverridesCommand)
                        .andThen(AutoBuilder.followPath(path))
                        .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);

            return beforeStarting
                    .andThen(resetOverridesCommand)
                    .andThen(AutoBuilder.pathfindThenFollowPath(
                            path,
                            new PathConstraints(
                                    MetersPerSecond.of(5),
                                    MetersPerSecondPerSecond.of(2),
                                    DegreesPerSecond.of(540),
                                    DegreesPerSecondPerSecond.of(720))))
                    .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * {@link AutoCommands#followPath(String, boolean, boolean, RobotState)} but mirrors the
     * path from left to right or vice versa.
     */
    private static Command followPathMirrored(String pathName, boolean resetPose, boolean resetOverrides, RobotState state) {
        if (resetOverrides) PPHolonomicDriveController.clearFeedbackOverrides();

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            Command resetOverridesCommand = Commands.runOnce(() -> {
                if (resetOverrides) PPHolonomicDriveController.setRotationTargetOverride(null);
            });
            Command beforeStarting = Commands.none();
            if (resetPose) {
                beforeStarting = Commands.runOnce(
                        () -> state.resetPose(path.getStartingHolonomicPose().get()));
            }
            var lastTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();

            if (resetPose)
                return beforeStarting
                        .andThen(resetOverridesCommand)
                        .andThen(AutoBuilder.followPath(path))
                        .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
            return beforeStarting
                    .andThen(resetOverridesCommand)
                    .andThen(AutoBuilder.pathfindThenFollowPath(
                            path,
                            new PathConstraints(
                                    MetersPerSecond.of(5),
                                    MetersPerSecondPerSecond.of(2),
                                    DegreesPerSecond.of(540),
                                    DegreesPerSecondPerSecond.of(720))))
                    .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private static Command setRotationOverride(Drive drive, RobotState state) {
        return Commands.runOnce(() -> {
            PPHolonomicDriveController.clearFeedbackOverrides();
            PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(Rotation2d.fromRotations(state.getScoringSetpoint(drive).drivetrainRotations())));
        });
    }
}
