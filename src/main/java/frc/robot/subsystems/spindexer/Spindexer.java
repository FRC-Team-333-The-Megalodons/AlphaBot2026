package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public double get_spin_voltage() {
    // For the beginning of the second, we spin fast; for the rest of the second, we spin slow.
    // This number indicates the amount of time that we're spinning fast vs slow.
    // For example, if this is 0.9, it means 90% of the time we're spinning fast, and 10% slow.
    final double FAST_RATIO = 1.0;
    // For now, leave it always spinning fast. Put down to 0.9 to make it 90% of the time.

    final long cutoff_ms = (long) (1000.0 * FAST_RATIO);
    final long current_ms = System.currentTimeMillis() % 1000;

    if (current_ms >= cutoff_ms) {
      return SpindexerConstants.SPIN_VOLTAGE_SLOW;
    }
    return SpindexerConstants.SPIN_VOLTAGE;
  }

  public Command spin() {
    return runEnd(
        () -> io.setVoltage(get_spin_voltage()),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }

  public Command eject() {
    return runEnd(
        () -> io.setVoltage(SpindexerConstants.REVERSE_VOLTAGE),
        () -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE_STOPPED));
  }
}
