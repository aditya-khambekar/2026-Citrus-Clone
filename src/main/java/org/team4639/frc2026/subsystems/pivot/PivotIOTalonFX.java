/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Commands2;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO{
    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;
    // requests
    private final VoltageOut voltageOut;
    private final PositionVoltage positionVoltage;
    // StatusSignals
    private final StatusSignal<Angle> mechanismRotations;
    private final StatusSignal<AngularVelocity> mechanismRotationsPerSecond;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> amps;
    private final StatusSignal<Temperature> celsius;

    private final PIDController controller;

    public PivotIOTalonFX(PortConfiguration portConfiguration) {
        this.pivotMotor = Phoenix6Factory.createDefaultTalon(portConfiguration.intakePivot);
        this.pivotEncoder = Phoenix6Factory.createCANcoder(portConfiguration.intakePivotEncoder);

        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(PivotConfigs.pivotMotorConfig));
        this.voltageOut = new VoltageOut(0);
        this.positionVoltage = new PositionVoltage(0);

        mechanismRotations = pivotMotor.getPosition();
        mechanismRotationsPerSecond = pivotMotor.getVelocity();
        volts = pivotMotor.getMotorVoltage();
        amps = pivotMotor.getTorqueCurrent();
        celsius = pivotMotor.getDeviceTemp();

        RobotState.disabled.onTrue(Commands2.action(() -> PhoenixUtil.tryUntilOk(5, () -> pivotMotor.setNeutralMode(NeutralModeValue.Coast))));
        RobotState.disabled.onFalse(Commands2.action(() -> PhoenixUtil.tryUntilOk(5, () -> pivotMotor.setNeutralMode(NeutralModeValue.Brake))));

        this.controller = new PIDController(0, 0, 0);
        SmartDashboard.putData("Pivot PID", controller);
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setSetpointEncoderRotations(double encoderRotations) {
        setVoltage(controller.calculate(pivotEncoder.getAbsolutePosition().getValueAsDouble(), encoderRotations));
    }

    @Override
    public void setPositionMechanismRotations(double mechanismRotations) {
        pivotMotor.setPosition(mechanismRotations);
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        pivotMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                mechanismRotations,
                mechanismRotationsPerSecond,
                volts,
                amps,
                celsius
        ).isOK();

        inputs.mechanismRotations = mechanismRotations.getValueAsDouble();
        inputs.amps = amps.getValueAsDouble();
        inputs.volts = volts.getValueAsDouble();
        inputs.mechanismRotationsPerSecond = mechanismRotationsPerSecond.getValueAsDouble();
        inputs.celsius = celsius.getValueAsDouble();
        inputs.encoderRotations = pivotEncoder.getAbsolutePosition(true).getValueAsDouble();
    }
}
