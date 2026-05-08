/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.Constants;

public class FeederConfigs {
    public static TalonFXConfiguration feederConfig = createFeederConfig();

    private static TalonFXConfiguration createFeederConfig() {
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = 80;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40;

        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // towards drum is positive
        feederConfig.Feedback.SensorToMechanismRatio = 1.0 / FeederConstants.MOTOR_TO_FEEDER_REDUCTION;

        feederConfig.Audio.BeepOnConfig = false;

        feederConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        feederConfig.MotorOutput.PeakReverseDutyCycle = 0;

        feederConfig.Slot0.kP = 99999;
        feederConfig.Slot0.kI = 0;
        feederConfig.Slot0.kD = 0;
        feederConfig.Slot0.kS = 0;
        feederConfig.Slot0.kV = Constants.RobotConstants.THEORETICAL_X60_KV / FeederConstants.MOTOR_TO_FEEDER_REDUCTION;
        feederConfig.Slot0.kA = 0;

        return feederConfig;
    }
}
