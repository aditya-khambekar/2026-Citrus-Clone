/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

public class IntakeRollers extends FullSubsystem {
  private final IntakeRollersIO rollersIO;
  private final IntakeRollersIOInputsAutoLogged rollerInputs =
      new IntakeRollersIOInputsAutoLogged();

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

  private final RobotState state;

  @Setter private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public IntakeRollers(IntakeRollersIO rollersIO, RobotState state) {
    this.rollersIO = rollersIO;
    this.state = state;
    rollersIO.updateInputs(rollerInputs);
    setDefaultCommand(run(this::runStateMachine));

    Logger.recordOutput("IntakeRollers/SystemState", systemState.toString());
  }

  @Override
  public void periodicBeforeScheduler() {
    rollersIO.updateInputs(rollerInputs);
    Logger.processInputs("IntakeRollers", rollerInputs);
  }

  @Override
  public void periodicAfterScheduler() {
    state.acceptCANMeasurement(rollerInputs.leftConnected);
    state.acceptCANMeasurement(rollerInputs.rightConnected);

    state.acceptTemperatureMeasurement(rollerInputs.leftCelsius);
    state.acceptTemperatureMeasurement(rollerInputs.rightCelsius);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("IntakeRollers/SystemState", newState.toString());
      systemState = newState;
    }

    switch (systemState) {
      case IDLE -> handleIdle();
      case INTAKE -> handleIntake();
      case OUTTAKE -> handleOuttake();
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
    rollersIO.setVoltage(0);
  }

  public void handleIntake() {
    System.out.println("handnle intake");
    rollersIO.setSetpointMechanismRotationsPerSecond(IntakeRollersConstants.INTAKE_MECHANISM_RPS);
  }

  public void handleOuttake() {
    rollersIO.setSetpointMechanismRotationsPerSecond(-IntakeRollersConstants.INTAKE_MECHANISM_RPS);
  }
}
