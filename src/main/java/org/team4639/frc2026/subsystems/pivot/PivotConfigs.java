/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class PivotConfigs {
    public static TalonFXConfiguration pivotMotorConfig = createPivotConfig();

    private static TalonFXConfiguration createPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = 1.0 / PivotConstants.MOTOR_TO_ENCODER_REDUCTION;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = 50;

        // up is negative, down is positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Audio.BeepOnConfig = true;

        config.Slot0.kP = PivotConstants.kP;

        return config;
    }

    private static CANcoderConfiguration createEncoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        return config;
    }
}
