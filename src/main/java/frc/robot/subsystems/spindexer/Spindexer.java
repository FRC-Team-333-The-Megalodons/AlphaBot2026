package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Characterizable;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase implements Characterizable {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
    Logger.recordOutput("Spindexer/Voltage", inputs.appliedVolts);
    Logger.recordOutput("Spindexer/VelocityRPM", inputs.velocityRps * 60.0);
  }

  public Command spin() {
    return runEnd(
        () -> io.moveTo(SpindexerConstants.SPIN_RPM),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public Command spinSlow() {
    return runEnd(
        () -> io.moveTo(SpindexerConstants.SPIN_RPM_SLOW),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public Command eject() {
    return runEnd(
        () -> io.moveTo(SpindexerConstants.EJECT_RPM),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public Command spinAt(double rpm) {
    return runEnd(
        () -> io.moveTo(rpm), () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public boolean atTarget(double rpm) {
    return io.atTarget(rpm);
  }

  public Command spinVoltage() {
    return runEnd(
        () -> io.setVoltage(getSpinVoltage()),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public Command ejectVoltage() {
    return runEnd(
        () -> io.setVoltage(SpindexerConstants.REVERSE_VOLTAGE),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  private double getSpinVoltage() {
    final double FAST_RATIO = 1.0;
    final long cutoff_ms = (long) (1000.0 * FAST_RATIO);
    final long current_ms = System.currentTimeMillis() % 1000;
    if (current_ms >= cutoff_ms) {
      return SpindexerConstants.SPIN_VOLTAGE_SLOW;
    }
    return SpindexerConstants.SPIN_VOLTAGE;
  }

  @Override
  public Command characterize() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(7), null, null),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) -> {
                  log.motor("spindexer-sysid")
                      .voltage(Volts.of(inputs.appliedVolts))
                      .angularVelocity(RotationsPerSecond.of(inputs.velocityRps));
                },
                this));

    return Commands.sequence(
        Commands.print("Starting Spindexer SysId"),
        runSysIdSequence(routine),
        Commands.print("Spindexer SysId Completed"));
  }
}
