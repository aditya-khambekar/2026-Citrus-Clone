/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public sealed class FeederSysID {
    @Getter
    private SysIdRoutine routine;

    public static final class FeederSysIDCTRE extends FeederSysID {
        public FeederSysIDCTRE(Feeder feeder){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            feeder::setVoltage,
                            null, // SignalLogger handles logging
                            feeder
                    )
            );
        }
    }

    public static final class FeederSysIDWPI extends FeederSysID {
        public FeederSysIDWPI(Feeder feeder, FeederIO.FeederIOInputs inputs){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            feeder::setVoltage,
                            log -> {
                                log.motor("Feeder")
                                        .angularPosition(Rotations.of(inputs.motorPosition))
                                        .angularVelocity(Rotations.of(inputs.motorVelocity).per(Second))
                                        .voltage(Volts.of(inputs.motorVoltage));
                            }, // SignalLogger handles logging
                            feeder
                    )
            );
        }
    }
}