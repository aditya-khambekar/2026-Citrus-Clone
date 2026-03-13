/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.lib.util.VirtualSubsystem;

public class SimRobot extends VirtualSubsystem {
  @Getter private static volatile SimRobot instance = new SimRobot();

  @Setter @Getter private SwerveDriveSimulation swerveDriveSimulation = null;

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(Constants.RobotConstants.ROBOT_MASS_KG))
          .withCustomModuleTranslations(Drive.getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  Constants.RobotConstants.WHEEL_COF,
                  3))
          .withBumperSize(Inches.of(34), Inches.of(34));

  @Override
  public void periodic() {
    // DO nothing
  }

  @Override
  public void periodicAfterScheduler() {
    if (Constants.currentMode == Mode.SIM) { // Only do if it is simulation AND not replay
      SimulatedArena.getInstance().simulationPeriodic();
      Logger.recordOutput(
          "Sim/SimulatedDrivetrainPose", swerveDriveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput(
          "Sim/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
  }

  public void setupDriveSim() {
    this.swerveDriveSimulation =
        new SwerveDriveSimulation(mapleSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
    var arena = new Arena2026Rebuilt(false);
    arena.setEfficiencyMode(false);
    SimulatedArena.overrideInstance(arena);
    SimulatedArena.getInstance().addDriveTrainSimulation(this.swerveDriveSimulation);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void resetPose(Pose2d pose) {
    this.swerveDriveSimulation.setSimulationWorldPose(pose);
  }
}
