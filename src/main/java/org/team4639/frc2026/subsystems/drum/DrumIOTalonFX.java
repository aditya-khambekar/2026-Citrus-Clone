/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class DrumIOTalonFX implements DrumIO {
    private final TalonFX[] motors; //0->left top, 1->left bottom, 2->right top, 3->right bottom
    private final TalonFX leader;

    private final VoltageOut voltageOut;
    private final VelocityVoltage velocityVoltage;

    public DrumIOTalonFX(PortConfiguration ports) {
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
            int I = i;
            PhoenixUtil.tryUntilOk(5, () -> motors[I].getConfigurator().apply(DrumConfigs.drumConfig));
            DrumConfigs.configureStatusSignals(motors[i]);
        }

        leader = motors[0]; // left top

        motors[1].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Aligned));
        motors[2].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Opposed));
        motors[3].setControl(new Follower(motors[2].getDeviceID(), MotorAlignmentValue.Opposed));

        voltageOut = new VoltageOut(0);
        velocityVoltage = new VelocityVoltage(0);
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
            inputs.volts[i] = motor.getMotorVoltage().getValueAsDouble();
            inputs.amps[i] = motor.getSupplyCurrent().getValueAsDouble();
            inputs.celsius[i] = motor.getDeviceTemp().getValueAsDouble();
            inputs.mechanismRPM[i] = (motor.getVelocity().getValueAsDouble() * 60);
            inputs.mechanismRotations[i] = motor.getPosition().getValueAsDouble();
            inputs.mechanismRPMPerSecond[i] = motor.getAcceleration().getValueAsDouble() * 60;
        }
    }

    @Override
    public void setVoltage(double appliedVolts) {
        voltageOut.Output = appliedVolts;
        leader.setControl(voltageOut);
    }

    @Override
    public void setSetpointMechanismRPM(double mechanismRPM) {
        setSetpointMechanismRPM(mechanismRPM, 0);
    }

    @Override
    public void setSetpointMechanismRPM(double mechanismRPM, double mechanismRPMPerSecond) {
        velocityVoltage.Velocity = mechanismRPM / 60;
        velocityVoltage.Acceleration = mechanismRPMPerSecond / 60;
        leader.setControl(velocityVoltage);
    }
}