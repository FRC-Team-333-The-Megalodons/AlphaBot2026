package frc.robot.subsystems.transfer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
  private final TransferIO io;
  private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public Transfer(TransferIO io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null, null),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) -> {
                  log.motor("transfer-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(RotationsPerSecond.of(inputs.velocityRpm / 60.0));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);
  }

  public void runVelocity() {
    io.moveTo(TransferConstants.TARGET_RPM);
  }

  public void stop() {
    io.setVoltage(0.0);
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
