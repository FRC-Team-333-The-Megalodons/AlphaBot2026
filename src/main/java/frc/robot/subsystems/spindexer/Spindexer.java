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

  public Command spinAt(double rpm, boolean waitForCompletion) {
    Runnable func = () -> io.moveTo(rpm);
    Command com = waitForCompletion ? run(func).until(() -> io.atTarget(rpm)) : runOnce(func);
    return com.handleInterrupt(this::stop);
  }

  public Command spin() {
    return runEnd(this::run, this::stop);
  }

  private void run() {
    io.setVoltage(SpindexerConstants.MOTOR_VOLTS);
  }

  private void stop() {
    ;
  }
}
