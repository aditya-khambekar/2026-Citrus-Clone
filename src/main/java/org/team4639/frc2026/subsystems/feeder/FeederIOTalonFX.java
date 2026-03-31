/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX feederMotor;

    private final VoltageOut voltageOut = new VoltageOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> amps;
    private final StatusSignal<AngularVelocity> mechanismRotationsPerSecond;
    private final StatusSignal<Angle> mechanismRotations;
    private final StatusSignal<Temperature> celsius;

    public FeederIOTalonFX(PortConfiguration ports) {
        feederMotor = Phoenix6Factory.createDefaultTalon(ports.feeder, false);

        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(FeederConfigs.feederConfig));

        this.volts = feederMotor.getMotorVoltage();
        this.amps = feederMotor.getTorqueCurrent();
        this.mechanismRotationsPerSecond = feederMotor.getVelocity();
        this.mechanismRotations = feederMotor.getPosition();
        this.celsius = feederMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                volts, amps, mechanismRotations, mechanismRotationsPerSecond, celsius
        ).isOK();
        inputs.volts = volts.getValueAsDouble();
        inputs.amps = amps.getValueAsDouble();
        inputs.mechanismRotationsPerSecond = mechanismRotationsPerSecond.getValueAsDouble();
        inputs.celsius = celsius.getValueAsDouble();
        inputs.mechanismRotations = mechanismRotations.getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        feederMotor.setControl(voltageOut.withOutput(appliedVoltage));
    }

    @Override
    public void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond) {
        setSetpointMechanismRotationsPerSecond(mechanismRotationsPerSecond, 0);
    }

    @Override
    public void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond, double mechanismRotationsPerSecondPerSecond) {
        velocityVoltage.Velocity = mechanismRotationsPerSecond;
        velocityVoltage.Acceleration = mechanismRotationsPerSecondPerSecond;

        feederMotor.setControl(velocityVoltage);
    }
}