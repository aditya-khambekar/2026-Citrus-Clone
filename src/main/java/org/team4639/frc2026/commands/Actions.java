/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.BLine.Path;
import lombok.Builder;
import org.json.simple.parser.ParseException;
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
import org.team4639.frc2026.subsystems.verticalextension.VerticalExtension;

import java.io.IOException;
import java.util.function.Supplier;

import static org.team4639.lib.oi.OI.driver;

@Builder
public class Actions {
    private final Drive drive;
    private final Drum drum;
    private final Hood hood;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Pivot pivot;
    private final IntakeRollers intakeRollers;
    private final VerticalExtension verticalExtension;

    public Command joystickDrive() {
        return DriveCommands.joystickDriveWithX(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () ->
                        Math.pow(Math.abs(driver.getRightX()), 0.75) * (driver.getRightX() > 0 ? -1 : 1));
    }

    public Command idleSuperstructure() {
        return SuperstructureCommands.idle(drum, hood, feeder, hopper);
    }

    public Command teleopRequestScoring(){
        return new ParallelCommandGroup(
                DriveCommands.joystickDriveWhileScoring(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX()
                ),
                new SequentialCommandGroup(
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                .until(drum::aboveSetpoint),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )
                )
        );
    }

    public Command autoRequestScoring(Supplier<Translation2d> fieldRelativeDriveSpeeds){
        return new ParallelCommandGroup(
                DriveCommands.launchWithTranslationalSpeed(drive, fieldRelativeDriveSpeeds),
                new SequentialCommandGroup(
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                .until(drum::aboveSetpoint),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )
                )
        );
    }

    public Command autoRequestScoring(Translation2d fieldRelativeDriveSpeeds){
        return autoRequestScoring(()  -> fieldRelativeDriveSpeeds);
    }

    public Command autoRequestScoring() {
        return autoRequestScoring(Translation2d.kZero);
    }

    public Command teleopRequestPassing(){
        return new ParallelCommandGroup(
                DriveCommands.joystickDriveWhilePassing(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX()
                ),
                new SequentialCommandGroup(
                        SuperstructureCommands.passingSpinup(drum, hood, feeder, hopper)
                                .until(drum::aboveSetpoint),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.passingSpinup(drum, hood, feeder, hopper)
                                                .until(DriveCommands::atPassingGoal),
                                        SuperstructureCommands.pass(drum, hood, feeder, hopper)
                                                .until(() -> !DriveCommands.atPassingGoal())
                                )
                        )
                )
        );
    }

    public Command intake() {
        return SuperstructureCommands.intake(intakeRollers);
    }

    public Command stopIntake() {
        return SuperstructureCommands.stopIntake(intakeRollers);
    }

    public Command pivotDown() {
        return SuperstructureCommands.pivotDown(pivot);
    }

    public Command pivotUp() {
        return SuperstructureCommands.pivotUp(pivot);
    }

    public Command agitate() {
        return Commands.either(
                SuperstructureCommands.pivotDownUp(pivot),
                SuperstructureCommands.pivotUpDown(pivot),
                () -> RobotState.getInstance().isPivotUp
        );
    }

    public Command followPath(String pathName){
        try {
            var path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public Command followPathWithResetPose(String pathName){
        try {
            var path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path).beforeStarting(() -> RobotState.getInstance().resetPose(path.getStartingHolonomicPose().get()));
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public Command followPathMirrored(String pathName) {
        try {
            var path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            return AutoBuilder.followPath(path);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public Command followPathMirroredWithResetPose(String pathName) {
        try {
            var path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            return AutoBuilder.followPath(path).beforeStarting(() -> RobotState.getInstance().resetPose(path.getStartingHolonomicPose().get()));
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public Command followPath(Path path) {
        return Drive.pathBuilder.build(path);
    }

    public Command followPathMirrored(Path path) {
        var pathMirrored = path.copy();
        pathMirrored.mirror();
        return followPath(pathMirrored);
    }
}
