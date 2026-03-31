package org.team4639.frc2026.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class HopperConfigs {
    public static TalonFXConfiguration hopperConfig = getHopperConfig();

    private static TalonFXConfiguration getHopperConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // towards drum is positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Audio.BeepOnConfig = false;

        config.Feedback.SensorToMechanismRatio = 1.0 / HopperConstants.MOTOR_TO_ROLLER_REDUCTION;

        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.kP = 0;

        return config;
    }
}
