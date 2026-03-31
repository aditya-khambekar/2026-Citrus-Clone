package org.team4639.frc2026.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

        hoodConfig.Slot0.kP = 5.0 / HoodConstants.FULL_EXTENSION_ROTOR_ROTATIONS;
        hoodConfig.Slot0.kI = 0;
        hoodConfig.Slot0.kD = 0;
        hoodConfig.Slot0.kS = 0;
        hoodConfig.Slot0.kV = 0;
        hoodConfig.Slot0.kA = 0;

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // only for disabled, a trigger sets it on enable

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_LAUNCH_DEGREES;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_LAUNCH_DEGREES;

        return hoodConfig;
    }
}
