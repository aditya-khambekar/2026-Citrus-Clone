/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drum;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.subsystems.drum.DrumIO.ShooterIOInputs;

import static edu.wpi.first.units.Units.*;

@RequiredArgsConstructor
public abstract sealed class DrumSysID {
    private final Drum drum;
    private final ShooterIOInputs inputs;
    @Getter
    private SysIdRoutine routine;

    public static final class DrumSysIDWPI extends DrumSysID {
        public DrumSysIDWPI(Drum drum, ShooterIOInputs inputs){
            super(drum, inputs);
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(6),
                            Seconds.of(16),
                            (state) -> Logger.recordOutput("SysIdTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            drum::setVoltage,
                            log -> {
                                // default = REV, left is leader
                                log.motor("Shooter")
                                        .angularVelocity(Rotations.per(Minute).of(inputs.mechanismRPM[0]))
                                        .angularPosition(Rotations.of(inputs.mechanismRotations[0]))
                                        .voltage(Volts.of(inputs.volts[0]));
                            }
                            , drum)
            );
        }
    }

    public static final class DrumSysIDURCL extends DrumSysID {
        public DrumSysIDURCL(Drum drum, ShooterIOInputs inputs){
            super(drum, inputs);
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.25),
                            Volts.of(3),
                            null,
                            (state) -> Logger.recordOutput("SysIdTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            drum::setVoltage,
                            null // record URCL data, left motor should be used as leader for SparkFlex io
                            , drum)
            );
        }
    }

    public static final class DrumSysIDCTRE extends DrumSysID {
        public DrumSysIDCTRE(Drum drum, ShooterIOInputs inputs){
            super(drum, inputs);
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.25),
                            Volts.of(3),
                            null,
                            (state) -> SignalLogger.writeString("SysIdTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            drum::setVoltage,
                            null // record SignalLogger data, right motor should be used as leader in io talonFX
                            , drum)
            );
        }
    }
}