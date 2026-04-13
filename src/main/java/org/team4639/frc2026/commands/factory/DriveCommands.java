/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands.factory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.lib.util.LoggedTunableNumber;
import org.team4639.lib.util.geometry.GeomUtil;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 10.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double ALIGN_FF = 1.0;

    private static final PIDController anglePID = new PIDController(6, 0, 0.1);

    static {
        SmartDashboard.putData("Angle PID", anglePID);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private static final LoggedTunableNumber driveLauncherCORMinErrorDeg =
            new LoggedTunableNumber("COR Min", 15.0);
    private static final LoggedTunableNumber driveLauncherCORMaxErrorDeg =
            new LoggedTunableNumber("COR Max", 30.0);
    private static final LoggedTunableNumber driveYawLaunchToleranceDeg =
            new LoggedTunableNumber("Drive Tolerance", 10);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = false; // false since our field pose is relative to the alliance wall
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveWithX(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          if (MathUtil.isNear(0, speeds.vxMetersPerSecond, 1e-9)
              && MathUtil.isNear(0, speeds.vyMetersPerSecond, 1e-9)
              && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, 1e-9)) drive.stopWithX();
          else
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        },
        drive);
  }

  public static Command joystickDriveWhileScoring(
                  Drive drive,
          DoubleSupplier xSupplier,
          DoubleSupplier ySupplier
  ) {
      return drive.run(() -> {
          var translationalVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          runSOTM(drive, translationalVelocity);
      });
  }

    public static Command joystickDriveWhilePassing(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier
    ) {
        return drive.run(() -> {
            var translationalVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            runPOTM(drive, translationalVelocity);
        });
    }

    private static void runSOTM(Drive drive, Translation2d fieldRelativeLinearVelocity) {
        final var setpoint = RobotState.getInstance().getScoringSetpoint(drive);
        final var nextSetpoint = RobotState.getInstance().getNextScoringSetpoint(drive);

        var driveVelocity = (nextSetpoint.drivetrainRotations() - setpoint.drivetrainRotations()) / 0.02;

        var launcherPose = RobotState.getInstance().getEstimatedPose().transformBy(Constants.RobotConstants.ORIGIN_TO_DRUM);

        Logger.recordOutput(
                "SOTM Setpoint",
                new Pose2d(
                        launcherPose.getX(),
                        launcherPose.getY(),
                        Rotation2d.fromRotations(setpoint.drivetrainRotations())));

        double omegaOutput =
                (Math.abs(anglePID.getError()) < Units.degreesToRadians(15)
                        ? Units.rotationsToRadians(driveVelocity)
                        : 0)
                        + anglePID.calculate(
                        MathUtil.inputModulus(
                                RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                                -Math.PI,
                                Math.PI),
                        MathUtil.inputModulus(
                                Units.rotationsToRadians(setpoint.drivetrainRotations()), -Math.PI, Math.PI));

        // Apply chassis speeds
        double corScalar =
                MathUtil.clamp(
                        (Math.abs(
                                Rotation2d.fromRotations(setpoint.drivetrainRotations())
                                        .minus(RobotState.getInstance().getEstimatedPose().getRotation())
                                        .getDegrees())
                                - driveLauncherCORMinErrorDeg.get())
                                / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                        0.0,
                        1.0);
        Translation2d launcherToRobot =
                Constants.RobotConstants.ORIGIN_TO_DRUM.getTranslation().unaryMinus();
        ChassisSpeeds fieldRelativeSpeedsWithOffset =
                GeomUtil.transformVelocity(
                        new ChassisSpeeds(
                                fieldRelativeLinearVelocity.getX(),
                                fieldRelativeLinearVelocity.getY(),
                                omegaOutput),
                        launcherToRobot.times(1.0 - corScalar),
                        RobotState.getInstance().getEstimatedPose().getRotation().plus(Rotation2d.kZero));

        if (MathUtil.isNear(0, fieldRelativeSpeedsWithOffset.vxMetersPerSecond, 1e-1)
                && MathUtil.isNear(0, fieldRelativeSpeedsWithOffset.vyMetersPerSecond, 1e-1)
                && atScoringGoal())
            drive.stopWithX();
        else
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRelativeSpeedsWithOffset, drive.getRotation()));

        // Override robot setpoint speeds published by drive. We run our calculations using the
        // speeds that will ultimately be applied once we are using the full robot-to-launcher
        // transform. This prevents the setpoint from changing due to the shifting COR of the
        // robot.
        ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
                GeomUtil.transformVelocity(
                        new ChassisSpeeds(
                                fieldRelativeLinearVelocity.getX(),
                                fieldRelativeLinearVelocity.getY(),
                                omegaOutput),
                        launcherToRobot,
                        RobotState.getInstance().getEstimatedPose().getRotation());
        RobotState.getInstance()
                .setSetpointSpeeds(ChassisSpeeds.discretize(fieldRelativeSpeedsWithFullOffset, 0.02));
    }

    public static Command launchWithTranslationalSpeed(
            Drive drive, Supplier<Translation2d> fieldRelativeLinearVelocity) {
        return drive.run(() -> runSOTM(drive, fieldRelativeLinearVelocity.get()));
    }

    private static void runPOTM(Drive drive, Translation2d fieldRelativeLinearVelocity) {
        final var setpoint = RobotState.getInstance().getPassingSetpoint(drive);
        final var nextSetpoint = RobotState.getInstance().getNextPassingSetpoint(drive);

        var driveVelocity = (nextSetpoint.drivetrainRotations() - setpoint.drivetrainRotations()) / 0.02;

        var launcherPose = RobotState.getInstance().getEstimatedPose().transformBy(Constants.RobotConstants.ORIGIN_TO_DRUM);

        Logger.recordOutput(
                "SOTM Setpoint",
                new Pose2d(
                        launcherPose.getX(),
                        launcherPose.getY(),
                        Rotation2d.fromRotations(setpoint.drivetrainRotations())));

        double omegaOutput =
                (Math.abs(anglePID.getError()) < Units.degreesToRadians(15)
                        ? Units.rotationsToRadians(driveVelocity)
                        : 0)
                        + anglePID.calculate(
                        MathUtil.inputModulus(
                                RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                                -Math.PI,
                                Math.PI),
                        MathUtil.inputModulus(
                                Units.rotationsToRadians(setpoint.drivetrainRotations()), -Math.PI, Math.PI));

        // Apply chassis speeds
        double corScalar =
                MathUtil.clamp(
                        (Math.abs(
                                Rotation2d.fromRotations(setpoint.drivetrainRotations())
                                        .minus(RobotState.getInstance().getEstimatedPose().getRotation())
                                        .getDegrees())
                                - driveLauncherCORMinErrorDeg.get())
                                / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                        0.0,
                        1.0);
        Translation2d launcherToRobot =
                Constants.RobotConstants.ORIGIN_TO_DRUM.getTranslation().unaryMinus();
        ChassisSpeeds fieldRelativeSpeedsWithOffset =
                GeomUtil.transformVelocity(
                        new ChassisSpeeds(
                                fieldRelativeLinearVelocity.getX(),
                                fieldRelativeLinearVelocity.getY(),
                                omegaOutput),
                        launcherToRobot.times(1.0 - corScalar),
                        RobotState.getInstance().getEstimatedPose().getRotation().plus(Rotation2d.kZero));

        if (MathUtil.isNear(0, fieldRelativeSpeedsWithOffset.vxMetersPerSecond, 1e-1)
                && MathUtil.isNear(0, fieldRelativeSpeedsWithOffset.vyMetersPerSecond, 1e-1)
                && atPassingGoal())
            drive.stopWithX();
        else
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRelativeSpeedsWithOffset, drive.getRotation()));

        // Override robot setpoint speeds published by drive. We run our calculations using the
        // speeds that will ultimately be applied once we are using the full robot-to-launcher
        // transform. This prevents the setpoint from changing due to the shifting COR of the
        // robot.
        ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
                GeomUtil.transformVelocity(
                        new ChassisSpeeds(
                                fieldRelativeLinearVelocity.getX(),
                                fieldRelativeLinearVelocity.getY(),
                                omegaOutput),
                        launcherToRobot,
                        RobotState.getInstance().getEstimatedPose().getRotation());
        RobotState.getInstance()
                .setSetpointSpeeds(ChassisSpeeds.discretize(fieldRelativeSpeedsWithFullOffset, 0.02));
    }

    public static boolean atScoringGoal() {
        return DriverStation.isEnabled()
                && Math.abs(
                RobotState.getInstance()
                        .getEstimatedPose()
                        .getRotation()
                        .minus(
                                Rotation2d.fromRotations(
                                        RobotState.getInstance().getScoringSetpoint(3).drivetrainRotations()))
                        .getRadians())
                <= Units.degreesToRadians(driveYawLaunchToleranceDeg.get());
    }

    public static boolean atPassingGoal() {
        return DriverStation.isEnabled()
                && Math.abs(
                RobotState.getInstance()
                        .getEstimatedPose()
                        .getRotation()
                        .minus(
                                Rotation2d.fromRotations(
                                        RobotState.getInstance().getScoringSetpoint(3).drivetrainRotations()))
                        .getRadians())
                <= Units.degreesToRadians(driveYawLaunchToleranceDeg.get());
    }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped = false;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command PIDToPose(
      Drive drive,
      RobotState state,
      Pose2d destinationPose,
      double toleranceMeters,
      double velocityScaling) {
    ProfiledPIDController pidX =
        new ProfiledPIDController(
            3, 0, 0, new TrapezoidProfile.Constraints(5 * velocityScaling, 6));
    ProfiledPIDController pidY =
        new ProfiledPIDController(
            3, 0, 0, new TrapezoidProfile.Constraints(5 * velocityScaling, 6));
    PIDController headingController = new PIDController(5, 0, 0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setSetpoint(destinationPose.getRotation().getRadians());

    var startingPose = state.getEstimatedPose();

    pidX.reset(startingPose.getX(), state.getChassisSpeeds().vxMetersPerSecond);
    pidX.setGoal(destinationPose.getX());

    pidY.reset(startingPose.getY(), state.getChassisSpeeds().vyMetersPerSecond);
    pidY.setGoal(destinationPose.getY());

    double kp = 6;

    return drive
        .run(
            () -> {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          ALIGN_FF * pidX.getSetpoint().velocity
                              + pidX.calculate(drive.getPose().getX()),
                          ALIGN_FF * pidY.getSetpoint().velocity
                              + pidY.calculate(drive.getPose().getY()),
                          headingController.calculate(drive.getPose().getRotation().getRadians())),
                      drive.getPose().getRotation()));

              state.setChoreoSetpoint(destinationPose);
            })
        .until(
            () ->
                state
                        .getEstimatedPose()
                        .getTranslation()
                        .getDistance(destinationPose.getTranslation())
                    < toleranceMeters);
  }

  public static Command PIDToPose(
      Drive drive, RobotState state, Pose2d destinationPose, double toleranceMeters) {
    return PIDToPose(drive, state, destinationPose, toleranceMeters, 1.0);
  }
}
