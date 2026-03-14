/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.lib.util.FullSubsystem;

public class IntakeRollers extends FullSubsystem {
  private final IntakeRollersIO rollersIO;
  private final IntakeRollersIOInputsAutoLogged rollerInputs =
      new IntakeRollersIOInputsAutoLogged();

  private final int INTAKE_ROTOR_VELOCITY = 0;

  public enum WantedState {
    IDLE,
    INTAKE,
    OUTTAKE
  }

  public enum SystemState {
    IDLE,
    INTAKE,
    OUTTAKE
  }

  @Setter private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public IntakeRollers(IntakeRollersIO rollersIO) {
    this.rollersIO = rollersIO;
    rollersIO.updateInputs(rollerInputs);
    setDefaultCommand(run(this::runStateMachine));

    Logger.recordOutput("Intake/SystemState", systemState.toString());
  }

  @Override
  public void periodicBeforeScheduler() {
    rollersIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake Rollers", rollerInputs);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Intake/SystemState", newState.toString());
      systemState = newState;
    }

    switch (systemState) {
      case IDLE -> handleIdle();
      case INTAKE -> handleIntaking();
      case OUTTAKE -> handleOuttaking();
    }
  }

  public SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case INTAKE -> SystemState.INTAKE;
      case OUTTAKE -> SystemState.OUTTAKE;
    };
  }

  public void handleIdle() {
    rollersIO.stop();
  }

  public void handleIntaking() {
    rollersIO.setRotorVelocity(INTAKE_ROTOR_VELOCITY);
  }

  public void handleOuttaking() {
    rollersIO.setRotorVelocity(-INTAKE_ROTOR_VELOCITY);
  }
}
