/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.launch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.AbstractMap;

import edu.wpi.first.math.util.Units;
import org.team4639.frc2026.Constants;
import org.team4639.lib.util.geometry.GeomUtil;

public class LookupTables {
    private static final double PHASE_DELAY = 0.0;
    private static final double TOF_FUDGE = 1.00;

    public static final double MIN_RPM = Double.NEGATIVE_INFINITY;
    public static final double MAX_RPM = Double.POSITIVE_INFINITY;


    public static final InterpolatingDoubleTreeMap scoringDistanceToRPM =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(2.5, 1680.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 1750.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.0, 1800.0)
            );

    public static final InterpolatingDoubleTreeMap scoringDistanceToHoodDegrees =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(2.5, 22.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 23.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.0, 25.0));

    public static final InterpolatingDoubleTreeMap scoringDistanceToTOF =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.77, 1.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 1.25),
                    new AbstractMap.SimpleImmutableEntry<>(2.4, 0.93),
                    new AbstractMap.SimpleImmutableEntry<>(2.7, 0.97),
                    new AbstractMap.SimpleImmutableEntry<>(3.1, 1.03),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 1.07),
                    new AbstractMap.SimpleImmutableEntry<>(3.7, 1.11),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 1.14),
                    new AbstractMap.SimpleImmutableEntry<>(4.5, 1.19),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 1.21),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 1.27),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 1.35));

    public static final InterpolatingDoubleTreeMap passingDistanceToRPM =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.77, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.4, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.7, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.1, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.50, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.80, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.41, 1570.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 1650.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 1725.0),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 1800.0));

    public static final InterpolatingDoubleTreeMap passingDistanceToHoodDegrees =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.77, 15.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 19.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.4, 25.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.7, 27.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.1, 31.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 34.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.7, 37.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 40.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.5, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 43.0));

    public static final InterpolatingDoubleTreeMap passingDistanceToTOF =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.77, 1.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 1.25),
                    new AbstractMap.SimpleImmutableEntry<>(2.4, 0.93),
                    new AbstractMap.SimpleImmutableEntry<>(2.7, 0.97),
                    new AbstractMap.SimpleImmutableEntry<>(3.1, 1.03),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 1.07),
                    new AbstractMap.SimpleImmutableEntry<>(3.7, 1.11),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 1.14),
                    new AbstractMap.SimpleImmutableEntry<>(4.5, 1.19),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 1.21),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 1.27),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 1.35));

    public static LaunchSetpoint getScoringSetpoint(
            Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose) {
        return getLaunchSetpoint(currentRobotPose, fieldRelativeChassisSpeeds, targetPose, scoringDistanceToTOF, scoringDistanceToRPM, scoringDistanceToHoodDegrees);
    }

    public static LaunchSetpoint getPassingSetpoint(
            Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose) {
        return getLaunchSetpoint(currentRobotPose, fieldRelativeChassisSpeeds, targetPose, passingDistanceToTOF, passingDistanceToRPM, passingDistanceToHoodDegrees);
    }

    private static LaunchSetpoint getLaunchSetpoint(Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose, InterpolatingDoubleTreeMap distanceToTOF, InterpolatingDoubleTreeMap distanceToRPM, InterpolatingDoubleTreeMap distanceToHoodDegrees) {
        Pose2d nextEstimatedPose =
                currentRobotPose.exp(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                        fieldRelativeChassisSpeeds, currentRobotPose.getRotation())
                                .toTwist2d(PHASE_DELAY));
        ChassisSpeeds drumVelocity =
                GeomUtil.transformVelocity(
                        fieldRelativeChassisSpeeds,
                        Constants.RobotConstants.ORIGIN_TO_DRUM.getTranslation(),
                        currentRobotPose.getRotation());
        Pose2d drumPosition =
                nextEstimatedPose.transformBy(
                        new Transform2d(
                                Constants.RobotConstants.ORIGIN_TO_DRUM.getTranslation(),
                                Rotation2d.kZero));

        double distanceMeters = nextEstimatedPose.getTranslation().getDistance(targetPose);
        double TOF = scoringDistanceToTOF.get(distanceMeters);

        Pose2d lookaheadPose = drumPosition;
        double lookaheadTurretToTargetDistance = distanceMeters;

        for (int i = 0; i < 20; i++) {
            TOF = scoringDistanceToTOF.get(lookaheadTurretToTargetDistance);
            double offsetX = drumVelocity.vxMetersPerSecond * TOF;
            double offsetY = drumVelocity.vyMetersPerSecond * TOF;

            lookaheadPose =
                    new Pose2d(
                            drumPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                            drumPosition.getRotation());
            lookaheadTurretToTargetDistance = targetPose.getDistance(lookaheadPose.getTranslation());
        }

        double drivetrainRotations =
                MathUtil.inputModulus(
                        targetPose.minus(lookaheadPose.getTranslation()).getAngle().getRotations(), 0, 1);
        double drumRPM =
                scoringDistanceToRPM.get(lookaheadTurretToTargetDistance);
        double hoodDegrees = scoringDistanceToHoodDegrees.get(lookaheadTurretToTargetDistance);

        return new LaunchSetpoint(drivetrainRotations, Units.degreesToRotations(hoodDegrees), drumRPM);
    }
}
