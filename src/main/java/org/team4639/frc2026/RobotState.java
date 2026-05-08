/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.constants.launch.LaunchSetpoint;
import org.team4639.frc2026.constants.launch.LookupTables;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.vision.Vision.VisionConsumer;
import org.team4639.frc2026.util.ValueCacher;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.util.LoggedTunableNumber;
import org.team4639.lib.util.PoseEstimator;
import org.team4639.lib.util.VirtualSubsystem;
import org.team4639.lib.util.geometry.AllianceFlipUtil;
import org.team4639.lib.util.geometry.GeomUtil;

/**
 * RobotState handles all information involving the current state of the robot.
 *
 * <p>Pose: all poses are reported relative to *our* alliance wall, not the blue alliance wall. This
 * is done to simplify internal calculations, however there are methods available to access the true
 * on-field pose mainly for interplay with vision, but they should be used sparingly.
 */
@ExtensionMethod(GeomUtil.class)
public class RobotState extends VirtualSubsystem implements VisionConsumer {

  // -------------------------------------------------------------------------
  // Singleton
  // -------------------------------------------------------------------------

  private static RobotState instance = new RobotState();

  public static synchronized RobotState getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, RobotState::new);
  }

  // -------------------------------------------------------------------------
  // Constants & Configuration
  // -------------------------------------------------------------------------

  double poseBufferSizeSec = 2.0;

  // SmartDashboard / Logger keys
  private final String ROBOT_FIELD_INTERNAL_KEY = "/Internal/Robot Pose";
  private final String ROBOT_FIELD_TRUE_KEY = "/RobotState/Robot Pose";
  private final String CHOREO_SETPOINT_KEY = "/Internal/Choreo Setpoint";

  // -------------------------------------------------------------------------
  // Pose Buffers
  // -------------------------------------------------------------------------

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private final TimeInterpolatableBuffer<Pose2d> choreoSetpoints =
      TimeInterpolatableBuffer.createBuffer(0.05);

  // -------------------------------------------------------------------------
  // Odometry State
  // -------------------------------------------------------------------------

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Drive.getModuleTranslations());

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Assume gyro starts at zero. */
  private Rotation2d gyroOffset = Rotation2d.kZero;

  // -------------------------------------------------------------------------
  // Chassis Speeds
  // -------------------------------------------------------------------------

  @Getter private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  @Setter @Getter private double gyroRotationsPerSecond;

  // -------------------------------------------------------------------------
  // Scoring & Shooting State
  // -------------------------------------------------------------------------

  @Getter private double RPMFudge = 1;
  private final ValueCacher<Object, LaunchSetpoint> currentScoringSetpoint = new ValueCacher<>(() ->
    LookupTables.getScoringSetpoint(getSecondaryEstimatedPose(), getSetpointSpeeds(), FieldConstants.Hub.innerCenterPoint.toTranslation2d())
  );

  private final ValueCacher<Object, LaunchSetpoint> nextScoringSetpoint = new ValueCacher<>(() ->
          LookupTables.getScoringSetpoint(getSecondaryEstimatedPose().exp(ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), getSecondaryEstimatedPose().getRotation()).toTwist2d(0.02)), getSetpointSpeeds(), FieldConstants.Hub.innerCenterPoint.toTranslation2d())
  );

  private final ValueCacher<Object, LaunchSetpoint> currentPassingSetpoint = new ValueCacher<>(() ->
          LookupTables.getPassingSetpoint(getSecondaryEstimatedPose(), getSetpointSpeeds(), FieldConstants.Hub.innerCenterPoint.toTranslation2d())
  );

  private final ValueCacher<Object, LaunchSetpoint> nextPassingSetpoint = new ValueCacher<>(() ->
          LookupTables.getPassingSetpoint(getSecondaryEstimatedPose().exp(ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), getSecondaryEstimatedPose().getRotation()).toTwist2d(0.02)), getSetpointSpeeds(), FieldConstants.Hub.innerCenterPoint.toTranslation2d())
  );

  private final LoggedTunableNumber desiredHoodDegrees = new LoggedTunableNumber("Desired Hood Degrees", 10);
  private final LoggedTunableNumber desiredShooterRPM = new LoggedTunableNumber("Desired Shooter RPM", 0);

  // -------------------------------------------------------------------------
  // Miscellaneous Robot State
  // -------------------------------------------------------------------------

  private final PoseEstimator primaryPoseEstimator = new PoseEstimator(poseBufferSizeSec);
  private final PoseEstimator secondaryPoseEstimator = new PoseEstimator(poseBufferSizeSec);

  @Setter @Getter
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  @Setter private boolean sendVisionToPrimaryPoseEstimator = true;

  @Getter
  @AutoLogOutput(key = "Intake Extension Fraction")
  private double intakeExtensionFraction = 0.0;

  @Accessors(fluent = true)
  @Getter
  private boolean useIntakeProtection = true;

  private final Queue<Boolean> canIsConnected = new LinkedList<>();
  private final Queue<Boolean> temperaturesAreFine = new LinkedList<>();

  public static final Trigger disabled = RobotModeTriggers.disabled();

  @Setter @Getter
  private double pivotMechanismRotations = 0;

  @Setter @Getter
  private double verticalExtensionProportion = 0;

  // -------------------------------------------------------------------------
  // SmartDashboard / Field Display Objects
  // -------------------------------------------------------------------------

  private final Field2d robotFieldInternal = new Field2d();
  private final Field2d robotFieldTrue = new Field2d();

  // =========================================================================
  // Lifecycle Methods
  // =========================================================================

  @Override
  public void periodic() {
    robotFieldInternal.setRobotPose(getEstimatedPose());
    SmartDashboard.putData(ROBOT_FIELD_INTERNAL_KEY, robotFieldInternal);
    robotFieldTrue.setRobotPose(getTrueOnFieldPose());
    SmartDashboard.putData(ROBOT_FIELD_TRUE_KEY, robotFieldTrue);

    SmartDashboard.putBoolean(
        "CAN Measurements", canIsConnected.stream().allMatch(measurement -> measurement));
    SmartDashboard.putBoolean(
        "Motor Temperatures", temperaturesAreFine.stream().allMatch(measurement -> measurement));
    SmartDashboard.putNumber("Distance To Goal", getDistanceToGoal());

    canIsConnected.clear();
    temperaturesAreFine.clear();
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput(ROBOT_FIELD_INTERNAL_KEY, getEstimatedPose());
    Logger.recordOutput(ROBOT_FIELD_TRUE_KEY, getTrueOnFieldPose());
    Logger.recordOutput("/Internal/Secondary", getSecondaryEstimatedPose());
    if (!choreoSetpoints.getInternalBuffer().isEmpty()) {
      Logger.recordOutput(
          CHOREO_SETPOINT_KEY, choreoSetpoints.getInternalBuffer().lastEntry().getValue());
    }
  }

  // =========================================================================
  // Pose / Odometry Methods
  // =========================================================================

  public Pose2d getEstimatedPose() {
    return primaryPoseEstimator.getEstimatedPose();
  }

  public Pose2d getSecondaryEstimatedPose() {
    return secondaryPoseEstimator.getEstimatedPose();
  }

  /**
   * Returns the pose relative to the blue alliance wall. Should be used sparingly; for all internal
   * calculations, use {@link RobotState#getEstimatedPose()} instead.
   */
  public Pose2d getTrueOnFieldPose() {
    return AllianceFlipUtil.apply(getEstimatedPose());
  }

  public void resetPose(Pose2d pose) {
    primaryPoseEstimator.resetPose(pose);
    secondaryPoseEstimator.resetPose(pose);
    if (Constants.currentMode == Mode.SIM) SimRobot.getInstance().resetPose(pose);
  }

  public void resetGyro() {
    resetPose(getEstimatedPose().withRotation(new Rotation2d()));
  }

  public void addOdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {
    primaryPoseEstimator.addOdometryMeasurement(wheelPositions, gyroAngle, timestamp);
    secondaryPoseEstimator.addOdometryMeasurement(wheelPositions, gyroAngle, timestamp);
  }

  public void setChoreoSetpoint(Pose2d pose) {
    choreoSetpoints.addSample(Timer.getTimestamp(), pose);
  }

  public Pose3d[] getComponentPoses() {
    return new Pose3d[] {};
  }

  // =========================================================================
  // Vision Methods
  // =========================================================================

  @Override
  public void accept(
      int cameraIndex,
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    secondaryPoseEstimator.addVisionObservation(
        cameraIndex, AllianceFlipUtil.apply(visionRobotPoseMeters) , timestampSeconds, visionMeasurementStdDevs);
    if (sendVisionToPrimaryPoseEstimator)
      primaryPoseEstimator.addVisionObservation(
          cameraIndex, AllianceFlipUtil.apply(visionRobotPoseMeters), timestampSeconds, visionMeasurementStdDevs);
  }

  // =========================================================================
  // Chassis Speed Methods
  // =========================================================================

  public void updateChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  // =========================================================================
  // Hardware Health Methods
  // =========================================================================

  public void acceptCANMeasurement(boolean isConnected) {
    this.canIsConnected.add(isConnected);
  }

  public void acceptTemperatureMeasurement(double tempCelsius) {
    this.temperaturesAreFine.add(tempCelsius < 100);
  }

  // =========================================================================
  // LED Methods
  // =========================================================================

  public LEDPattern getDesiredLEDPattern() {
    return LEDPattern.BLANK;
  }

  // launch setpoints
  public LaunchSetpoint getScoringSetpoint(Object caller) {
    var setpoint = currentScoringSetpoint.get(caller);
    return new LaunchSetpoint(setpoint.drivetrainRotations(), setpoint.drumRotationsPerMinute(), setpoint.hoodRotations());
  }

  public LaunchSetpoint getNextScoringSetpoint(Object caller) {
      var setpoint = nextScoringSetpoint.get(caller);
      return new LaunchSetpoint(setpoint.drivetrainRotations(), setpoint.drumRotationsPerMinute(), setpoint.hoodRotations());
  }

  public LaunchSetpoint getPassingSetpoint(Object caller) {
    return currentPassingSetpoint.get(caller);
  }

  public LaunchSetpoint getNextPassingSetpoint(Object caller) {
    return nextPassingSetpoint.get(caller);
  }

  public double getDistanceToGoal() {
      return getEstimatedPose().transformBy(Constants.RobotConstants.ORIGIN_TO_DRUM).getTranslation().getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
  }
}
