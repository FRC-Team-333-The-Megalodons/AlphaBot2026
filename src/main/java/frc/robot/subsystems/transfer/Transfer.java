package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Characterizable;

import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase implements Characterizable {
  private final TransferIO io;
  private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

  public Transfer(TransferIO io) {
    this.io = io; 
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);
  }

  public Command feedShooter() {
    return runEnd(() -> io.setVoltage(TransferConstants.FEED_VOLTAGE), () -> io.setVoltage(0.0));
  }

  public Command feedShooterVelocity() {
    return runEnd(() -> io.moveTo(TransferConstants.TARGET_RPM), () -> io.setVoltage(0.0));
  }

  public boolean atTarget() {
    return io.atTarget(TransferConstants.TARGET_RPM);
  }

  @Override
  public Command characterize() {
    SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(7), null, null),
      new SysIdRoutine.Mechanism(
        (voltage) -> io.setVoltage(voltage.in(Volts)),
        (log) -> {
          log.motor("transfer-sysid")
            .voltage(Volts.of(inputs.appliedVolts))
            .angularVelocity(RotationsPerSecond.of(inputs.velocityRpm / 60.0));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.print("Starting Transfer SysId"),
      runSysIdSequence(routine),
      Commands.print("Transfer SysId Completed")
    );
  }
}
