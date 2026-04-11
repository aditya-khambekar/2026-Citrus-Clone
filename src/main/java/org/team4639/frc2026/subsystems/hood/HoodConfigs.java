package org.team4639.frc2026.subsystems.hood;

import org.team4639.frc2026.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class HoodConfigs {
    public static final TalonFXConfiguration hoodConfig = createHoodConfig();

    private static TalonFXConfiguration createHoodConfig() {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodConfig.Feedback.SensorToMechanismRatio = 1.0 / HoodConstants.MOTOR_TO_HOOD_REDUCTION;

        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 20;

        hoodConfig.Audio.BeepOnConfig = false;

        // positive moves the hood up
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.Slot0.kP = 200;
        hoodConfig.Slot0.kI = 0;
        hoodConfig.Slot0.kD = 4;
        hoodConfig.Slot0.kS = 0;
        hoodConfig.Slot0.kV = Constants.RobotConstants.THEORETICAL_X44_KV / HoodConstants.MOTOR_TO_HOOD_REDUCTION;
        hoodConfig.Slot0.kA = 0;

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // only for disabled, a trigger sets it on enable

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MAX_LAUNCH_DEGREES);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MIN_LAUNCH_DEGREES);

        return hoodConfig;
    }
}
