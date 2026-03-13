/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.constants.ports.WaterBottle;
import org.team4639.frc2026.subsystems.drive.*;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.vision.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.oi.DeadbandXboxController;
import org.team4639.lib.util.LoggedLazyAutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PortConfiguration portConfiguration = WaterBottle.portConfiguration;

  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Controller
  private final CommandXboxController driver = new DeadbandXboxController(0);
  private final CommandXboxController operator = new DeadbandXboxController(1);

  // Dashboard inputs
  private final LoggedLazyAutoChooser autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                pose -> {});

        vision =
            new Vision(
                RobotState.getInstance(),
                new VisionIOLimelight(
                    "limelight-left",
                    () -> RobotState.getInstance().getEstimatedPose().getRotation()),
                new VisionIOLimelight(
                    "limelight-right",
                    () -> RobotState.getInstance().getEstimatedPose().getRotation()));

        configureButtonBindings();
        break;

      case SIM:
        SimRobot.getInstance().setupDriveSim();

        drive =
            new Drive(
                new GyroIOSim(
                    SimRobot.getInstance().getSwerveDriveSimulation().getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontLeft,
                    SimRobot.getInstance().getSwerveDriveSimulation().getModules()[0]),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontRight,
                    SimRobot.getInstance().getSwerveDriveSimulation().getModules()[1]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackLeft,
                    SimRobot.getInstance().getSwerveDriveSimulation().getModules()[2]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackRight,
                    SimRobot.getInstance().getSwerveDriveSimulation().getModules()[3]),
                SimRobot.getInstance()::resetPose);

        // flip poses so that the vision sees the true on-field pose
        vision =
            new Vision(
                RobotState.getInstance(),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    () ->
                        SimRobot.getInstance()
                            .getSwerveDriveSimulation()
                            .getSimulatedDriveTrainPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    () ->
                        SimRobot.getInstance()
                            .getSwerveDriveSimulation()
                            .getSimulatedDriveTrainPose()));

        configureSimButtonBindings();
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                pose -> {});

        vision = new Vision(RobotState.getInstance());

        configureButtonBindings();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedLazyAutoChooser("Auto Choices");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithX(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () ->
                Math.pow(Math.abs(driver.getRightX()), 0.75) * (driver.getRightX() > 0 ? -1 : 1)));
  }

  private void configureSimButtonBindings() {
    configureButtonBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
