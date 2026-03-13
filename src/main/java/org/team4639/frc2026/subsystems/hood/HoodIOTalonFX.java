/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final PositionVoltage request = new PositionVoltage(0);

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    public HoodIOTalonFX(PortConfiguration ports) {
        hoodMotor = Phoenix6Factory.createDefaultTalon(ports.verticalExtension);

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // do NOT change this
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = 0;
        applyNewGains();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        motorVoltage = hoodMotor.getMotorVoltage();
        motorCurrent = hoodMotor.getStatorCurrent();
    }

    @Override
    public void setSetpointRotorRotations(double setpointRotorRotations) {
        request.Position = Units.degreesToRotations(setpointRotorRotations);
        hoodMotor.setControl(request);
        Logger.recordOutput("hopperExtension Controller Setpoint", request.Position);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodMotorConnected =
                BaseStatusSignal.refreshAll(
                                motorVoltage,
                                motorCurrent,
                                hoodMotor.getDeviceTemp(),
                                hoodVelocity,
                                hoodPosition)
                        .isOK();
        inputs.hoodVoltage = motorVoltage.getValueAsDouble();
        inputs.hoodCurrent = motorCurrent.getValueAsDouble();
        inputs.hoodTemperature = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.hoodPositionRotorRotations = hoodPosition.getValueAsDouble() * 360;
        inputs.hoodVelocityRotorRotations = hoodVelocity.getValueAsDouble() * 360;

        Logger.recordOutput("hopperExtension Rotations", hoodPosition.getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public void updateGains() {
        config.Slot0.kP = PIDs.hoodKp.get();
        config.Slot0.kI = PIDs.hoodKi.get();
        config.Slot0.kD = PIDs.hoodKd.get();
        config.Slot0.kS = PIDs.hoodKs.get();
        config.Slot0.kV = PIDs.hoodKv.get();
        config.Slot0.kA = PIDs.hoodKa.get();
    }

    @Override
    public void applyNewGains() {
        updateGains();
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));
    }

    @Override
    public void setPositionRotorRotations(double rotorRotations) {
        hoodMotor.setPosition(rotorRotations);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        hoodMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
