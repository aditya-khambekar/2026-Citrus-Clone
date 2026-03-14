/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopperfloor;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public sealed class HopperFloorSysID {
    @Getter
    private SysIdRoutine routine;

    public static final class HopperFloorSysIDCTRE extends HopperFloorSysID {
        public HopperFloorSysIDCTRE(HopperFloor hopperFloor){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            hopperFloor::setVoltage,
                            null, // SignalLogger handles logging
                            hopperFloor
                    )
            );
        }
    }

    public static final class HopperFloorSysIDWPI extends HopperFloorSysID {
        public HopperFloorSysIDWPI(HopperFloor hopperFloor, HopperFloorIO.HopperFloorIOInputs inputs){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            hopperFloor::setVoltage,
                            log -> {
                                log.motor("HopperFloor")
                                        .angularPosition(Rotations.of(inputs.motorPosition))
                                        .angularVelocity(Rotations.of(inputs.motorVelocity).per(Second))
                                        .voltage(Volts.of(inputs.motorVoltage));
                            }, // SignalLogger handles logging
                            hopperFloor
                    )
            );
        }
    }
}