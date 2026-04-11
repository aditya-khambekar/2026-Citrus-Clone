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
import org.team4639.frc2026.subsystems.drive.generated.TunerConstantsOverrides;
import org.team4639.frc2026.subsystems.drum.Drum;
import org.team4639.frc2026.subsystems.drum.DrumIO;
import org.team4639.frc2026.subsystems.drum.DrumIOTalonFX;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.hood.HoodIO;
import org.team4639.frc2026.subsystems.hood.HoodIOTalonFX;
import org.team4639.frc2026.subsystems.hopper.Hopper;
import org.team4639.frc2026.subsystems.hopper.HopperIO;
import org.team4639.frc2026.subsystems.hopper.HopperIOTalonFX;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollers;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollersIO;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollersIOTalonFX;
import org.team4639.frc2026.subsystems.intakeRollers.IntakeRollers.WantedState;
import org.team4639.frc2026.subsystems.vision.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.oi.DeadbandXboxController;
import org.team4639.lib.util.LoggedLazyAutoChooser;
import org.team4639.lib.util.SysIDUtils;
import org.team4639.lib.util.SysIDUtils.ButtonConfiguration;

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

  private final IntakeRollers intakeRollers;
  private final Drum drum;
  private final Hopper hopper;
  private final Hood hood;

  // Controller
  private final CommandXboxController driver = new DeadbandXboxController(0);
  private final CommandXboxController operator = new DeadbandXboxController(1);

  // Dashboard inputs
  private final LoggedLazyAutoChooser autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants.FrontLeft, TunerConstantsOverrides.overrides[0]),
        //         new ModuleIOTalonFX(TunerConstants.FrontRight, TunerConstantsOverrides.overrides[1]),
        //         new ModuleIOTalonFX(TunerConstants.BackLeft, TunerConstantsOverrides.overrides[2]),
        //         new ModuleIOTalonFX(TunerConstants.BackRight, TunerConstantsOverrides.overrides[3]),
        //         pose -> {});

        // vision =
        //     new Vision(
        //         RobotState.getInstance(),
        //         new VisionIOLimelight(
        //             "limelight-left",
        //             () -> RobotState.getInstance().getEstimatedPose().getRotation()),
        //         new VisionIOLimelight(
        //             "limelight-right",
        //             () -> RobotState.getInstance().getEstimatedPose().getRotation()));

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                pose -> {});

        vision = new Vision(RobotState.getInstance());

        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX(portConfiguration), RobotState.getInstance());
        drum = new Drum(new DrumIOTalonFX(portConfiguration), RobotState.getInstance());
        hopper = new Hopper(new HopperIOTalonFX(portConfiguration), RobotState.getInstance());
        hood = new Hood(new HoodIOTalonFX(portConfiguration), RobotState.getInstance());

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

                            intakeRollers = new IntakeRollers(new IntakeRollersIO() {}, RobotState.getInstance());
                            drum = new Drum(new DrumIO() {}, RobotState.getInstance());
                            hopper = new Hopper(new HopperIO() {
                                
                            }, RobotState.getInstance());
                            hood = new Hood(new HoodIO() {
                                
                            }, RobotState.getInstance());

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

        intakeRollers = new IntakeRollers(new IntakeRollersIO() {}, RobotState.getInstance());
        drum = new Drum(new DrumIO() {}, RobotState.getInstance());

        hopper = new Hopper(new HopperIO() {
                                
                            }, RobotState.getInstance());
                            hood = new Hood(new HoodIO() {
                                
                            }, RobotState.getInstance());

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

    driver.povUp().onTrue(Commands.runOnce(() -> hood.setWantedState(Hood.WantedState.SCORING)))
        .onFalse(Commands.runOnce(() -> hood.setWantedState(Hood.WantedState.IDLE)));
        SysIDUtils.bind(driver, ButtonConfiguration.XYAB, drum.getSysID().getRoutine());
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
