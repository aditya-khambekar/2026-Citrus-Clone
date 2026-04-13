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

    public static final InterpolatingDoubleTreeMap scoringDistanceToHoodDegrees =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.77, 15.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 19.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.4, 25.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.7, 28.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.1, 31.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.3, 34.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.7, 37.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 40.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.5, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 43.0),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 43.0));

    public static final InterpolatingDoubleTreeMap scoringDistanceToTOF =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.87, 0.8814285714 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.2, 0.9725 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.5, 1.0175 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.9, 1.035 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.2, 1.061428571 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.5, 1.176666667 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.8, 1.192 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.1, 1.238571429 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.4, 1.3425 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 1.42 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(5.2, 1.4725 * TOF_FUDGE));

    public static final InterpolatingDoubleTreeMap passingDistanceToRPM =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.87, 2320.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.20, 2520.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.44, 2690.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.90, 2825.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.20, 2930.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.50, 3015.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.80, 3100.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.10, 3260.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.41, 3370.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 3465.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 3635.0),
                    new AbstractMap.SimpleImmutableEntry<>(5.20, 3765.0),
                    // New Setpoints
                    new AbstractMap.SimpleImmutableEntry<>(5.5, 4005.3875),
                    new AbstractMap.SimpleImmutableEntry<>(5.8, 4280.9288),
                    new AbstractMap.SimpleImmutableEntry<>(6.1, 4611.5969),
                    new AbstractMap.SimpleImmutableEntry<>(6.5, 5150.8125),
                    new AbstractMap.SimpleImmutableEntry<>(6.8, 5638.3968),
                    new AbstractMap.SimpleImmutableEntry<>(7.1, 6205.3539),
                    new AbstractMap.SimpleImmutableEntry<>(7.4, 6858.9576));

    public static final InterpolatingDoubleTreeMap passingDistanceToHoodDegrees =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(0.0, 20.0),
                    new AbstractMap.SimpleImmutableEntry<>(6.0, 20.0),
                    new AbstractMap.SimpleImmutableEntry<>(6.00001, 35.0),
                    new AbstractMap.SimpleImmutableEntry<>(20.0, 35.0));

    public static final InterpolatingDoubleTreeMap passingDistanceToTOF =
            InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.87, 0.8814285714 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.2, 0.9725 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.5, 1.0175 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(2.9, 1.035 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.2, 1.061428571 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.5, 1.176666667 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(3.8, 1.192 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.1, 1.238571429 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.4, 1.3425 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(4.77, 1.42 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(5.2, 1.4725 * TOF_FUDGE),
                    // New Setpoints
                    new AbstractMap.SimpleImmutableEntry<>(5.5, 1.55855625 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(5.8, 1.629534 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(6.1, 1.70293275 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(6.5, 1.80449375 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(6.8, 1.883384 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(7.1, 1.96456025 * TOF_FUDGE),
                    new AbstractMap.SimpleImmutableEntry<>(7.4, 2.047982));

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
