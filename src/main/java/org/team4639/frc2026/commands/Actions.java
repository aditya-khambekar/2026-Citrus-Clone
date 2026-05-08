/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import lombok.Builder;
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
//                DriveCommands.joystickDriveWhileScoring(
//                        drive,
//                        () -> -driver.getLeftY(),
//                        () -> -driver.getLeftX()
//                ),
                new SequentialCommandGroup(
                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
                                .until(drum::aboveSetpoint),
                        new RepeatCommand(
                                new SequentialCommandGroup(
//                                        SuperstructureCommands.scoringSpinup(drum, hood, feeder, hopper)
//                                                .until(DriveCommands::atScoringGoal),
                                        SuperstructureCommands.score(drum, hood, feeder, hopper)
//                                                .until(() -> !DriveCommands.atScoringGoal())
                                )
                        )
                )
        );
    }

    public Command teleopRequestPassing(){
        return new ParallelCommandGroup(
//                DriveCommands.joystickDriveWhilePassing(
//                        drive,
//                        () -> -driver.getLeftY(),
//                        () -> -driver.getLeftX()
//                ),
                new SequentialCommandGroup(
                        SuperstructureCommands.passingSpinup(drum, hood, feeder, hopper)
                                .until(drum::aboveSetpoint),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        SuperstructureCommands.passingSpinup(drum, hood, feeder, hopper)
//                                                .until(DriveCommands::atPassingGoal),
//                                        SuperstructureCommands.pass(drum, hood, feeder, hopper)
//                                                .until(() -> !DriveCommands.atPassingGoal())
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

    public Command intakeExtend() {
        return SuperstructureCommands.extend(pivot);
    }

    public Command intakeRetract() {
        return SuperstructureCommands.retract(pivot);
    }
}
