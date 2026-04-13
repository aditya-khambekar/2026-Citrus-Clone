/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands.factory;

import edu.wpi.first.wpilibj2.command.*;
import lombok.Getter;
import org.team4639.frc2026.subsystems.drum.Drum;
import org.team4639.frc2026.subsystems.feeder.Feeder;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.hopper.Hopper;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollers;
import org.team4639.frc2026.subsystems.pivot.Pivot;

public class SuperstructureCommands {
    @Getter
    private static final SubsystemBase scoringDummy = new SubsystemBase() {};
    @Getter
    private static final SubsystemBase intakeDummy = new SubsystemBase() {};
    @Getter
    private static final SubsystemBase pivotDummy = new SubsystemBase() {};
    @Getter
    private static final SubsystemBase extensionDummy = new SubsystemBase() {};

    private static final double AGITATE_PERIOD = 1.0;

    public static Command idle(Drum drum, Hood hood, Feeder feeder, Hopper hopper) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drum.setWantedState(Drum.WantedState.IDLE);
                    hood.setWantedState(Hood.WantedState.IDLE);
                    feeder.setWantedState(Feeder.WantedState.IDLE);
                    hopper.setWantedState(Hopper.WantedState.IDLE);
                }, scoringDummy),
                Commands.idle(scoringDummy)
        );
    }
    public static Command scoringSpinup(Drum drum, Hood hood, Feeder feeder, Hopper hopper) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drum.setWantedState(Drum.WantedState.SCORING);
                    hood.setWantedState(Hood.WantedState.SCORING);
                    feeder.setWantedState(Feeder.WantedState.IDLE);
                    hopper.setWantedState(Hopper.WantedState.IDLE);
                }, scoringDummy),
                Commands.idle(scoringDummy)
        );
    }

    public static Command score(Drum drum, Hood hood, Feeder feeder, Hopper hopper) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drum.setWantedState(Drum.WantedState.SCORING);
                    hood.setWantedState(Hood.WantedState.SCORING);
                    feeder.setWantedState(Feeder.WantedState.FEED_SCORING);
                    hopper.setWantedState(Hopper.WantedState.ON);
                }, scoringDummy),
                Commands.idle(scoringDummy)
        );
    }

    public static Command passingSpinup(Drum drum, Hood hood, Feeder feeder, Hopper hopper) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drum.setWantedState(Drum.WantedState.PASSING);
                    hood.setWantedState(Hood.WantedState.PASSING);
                    feeder.setWantedState(Feeder.WantedState.IDLE);
                    hopper.setWantedState(Hopper.WantedState.IDLE);
                }, scoringDummy),
                Commands.idle(scoringDummy)
        );
    }

    public static Command pass(Drum drum, Hood hood, Feeder feeder, Hopper hopper) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drum.setWantedState(Drum.WantedState.PASSING);
                    hood.setWantedState(Hood.WantedState.PASSING);
                    feeder.setWantedState(Feeder.WantedState.FEED_PASSING);
                    hopper.setWantedState(Hopper.WantedState.ON);
                }, scoringDummy),
                Commands.idle(scoringDummy)
        );
    }

    public static Command intake(IntakeRollers intakeRollers) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeRollers.setWantedState(IntakeRollers.WantedState.INTAKE);
                }, intakeDummy),
                Commands.idle(intakeDummy)
        );
    }

    public static Command stopIntake(IntakeRollers intakeRollers) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeRollers.setWantedState(IntakeRollers.WantedState.IDLE);
                }, intakeDummy),
                Commands.idle(intakeDummy)
        );
    }

    public static Command outtake(IntakeRollers intakeRollers) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeRollers.setWantedState(IntakeRollers.WantedState.OUTTAKE);
                }, intakeDummy),
                Commands.idle(intakeDummy)
        );
    }

    public static Command pivotUp(Pivot pivot){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    pivot.setWantedState(Pivot.WantedState.IDLE);
                }, pivotDummy),
                Commands.idle(pivotDummy)
        );
    }

    public static Command pivotDown(Pivot pivot){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    pivot.setWantedState(Pivot.WantedState.DOWN);
                }, pivotDummy),
                Commands.idle(pivotDummy)
        );
    }

    public static Command pivotUpDown(Pivot pivot) {
        return new SequentialCommandGroup(
                pivotUp(pivot).withTimeout(AGITATE_PERIOD/2.0),
                pivotDown(pivot).withTimeout(AGITATE_PERIOD / 2.0)
        ).repeatedly();
    }

    public static Command pivotDownUp(Pivot pivot) {
        return new SequentialCommandGroup(
                pivotDown(pivot).withTimeout(AGITATE_PERIOD/2.0),
                pivotUp(pivot).withTimeout(AGITATE_PERIOD / 2.0)
        ).repeatedly();
    }
}
