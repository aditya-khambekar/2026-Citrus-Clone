package org.team4639.frc2026.subsystems.intakeRollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersConfigs {
    public static final TalonFXConfiguration leftConfig = getLeftConfig();
    public static final TalonFXConfiguration rightConfig = getRightConfig();

    private static TalonFXConfiguration getGeneralConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLowerTime = 1;
        config.CurrentLimits.SupplyCurrentLowerLimit = 20;
        config.CurrentLimits.StatorCurrentLimit = 40;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Feedback.SensorToMechanismRatio = 1.0 / IntakeRollersConstants.MOTOR_TO_ROLLER_REDUCTION;

        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        return config;
    }

    private static TalonFXConfiguration getLeftConfig() {
        TalonFXConfiguration config = getGeneralConfig();

        // this is the leader, we want this motor's positive output to correspond to
        // intaking
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    private static TalonFXConfiguration getRightConfig() {
        TalonFXConfiguration config = getGeneralConfig();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return config;
    }
}
