/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX[] motors; //0->left top, 1->left bottom, 2->right top, 3->right bottom
    private final TalonFX masterMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VoltageOut shooterVoltageControl = new VoltageOut(0);

    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0);

    private final double SHOOTER_GEAR_RATIO = 1.0;

    public ShooterIOTalonFX(PortConfiguration ports) {
        motors = new TalonFX[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = Phoenix6Factory.createDefaultTalon(
                    switch (i) {
                        case 0 -> ports.leftTopDrum;
                        case 1 -> ports.leftBottomDrum;
                        case 2 -> ports.rightTopDrum;
                        default -> ports.rightBottomDrum;
                    }
            );
        }
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        applyNewGains();

        masterMotor = motors[2];

        motors[0].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Opposed));
        motors[1].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Opposed));
        motors[3].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        for (int i = 0; i < 4; i++ ) {
            TalonFX motor = motors[i];
            inputs.connected[i] = BaseStatusSignal.refreshAll(
                            motor.getMotorVoltage(),
                            motor.getSupplyCurrent(),
                            motor.getDeviceTemp(),
                            motor.getVelocity())
                    .isOK();
            inputs.voltage[i] = motor.getMotorVoltage().getValueAsDouble();
            inputs.current[i] = motor.getSupplyCurrent().getValueAsDouble();
            inputs.temperature[i] = motor.getDeviceTemp().getValueAsDouble();
            inputs.RPM[i] = (motor.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
            inputs.rotations[i] = motor.getRotorPosition().getValueAsDouble() / SHOOTER_GEAR_RATIO;
        }
    }

    @Override
    public void setVoltage(double appliedVolts) {
        masterMotor.setControl(shooterVoltageControl.withOutput(appliedVolts));
    }

    @Override
    public void setRPM(double targetRPM) {
        double applied = targetRPM * SHOOTER_GEAR_RATIO / 60.0;
        masterMotor.setControl(shooterVelocityControl.withVelocity(applied));
    }

    public void updateGains() {
        config.Slot0.kP = PIDs.shooterKp.get();
        config.Slot0.kI = PIDs.shooterKi.get();
        config.Slot0.kD = PIDs.shooterKd.get();
        config.Slot0.kS = PIDs.shooterKs.get();
        config.Slot0.kV = PIDs.shooterKv.get();
        config.Slot0.kA = PIDs.shooterKa.get();
    }

    @Override
    public void applyNewGains() {
        updateGains();
        for (var motor : motors) {
            PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        }
    }
}