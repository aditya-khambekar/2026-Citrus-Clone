package org.team4639.frc2026.subsystems.verticalextension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class VerticalExtensionConfigs {
    public static final TalonFXConfiguration verticalExtensionConfig = createVerticalExtensionConfig();

    private static TalonFXConfiguration createVerticalExtensionConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimit = 20.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // not sure which way is the right one but, up should be positive so if we need to change this we change it
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return config;
    }
}
