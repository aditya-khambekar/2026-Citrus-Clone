package org.team4639.frc2026.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class PivotConfigs {
    public static TalonFXConfiguration pivotMotorConfig = createPivotConfig();

    private static TalonFXConfiguration createPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = 1.0 / PivotConstants.MOTOR_TO_PIVOT_REDUCTION;

        // up is negative, down is positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = 3.0 / 4.0;

        return config;
    }
}
