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

  public Command spin() {
    return runEnd(() -> io.setVoltage(SpindexerConstants.SPIN_VOLTAGE), () -> io.setVoltage(0.0));
  }

  public Command eject() {
    return runEnd(
        () -> io.setVoltage(SpindexerConstants.REVERSE_VOLTAGE), () -> io.setVoltage(0.0));
  }
}
