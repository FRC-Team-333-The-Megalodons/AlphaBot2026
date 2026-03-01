package frc.robot.subsystems.spindexer;

import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO extends Automatable {
  @AutoLog
  public static class SpindexerIOInputs {
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setVelocity(double rps) {}

  public default void setVoltage(double volts) {}
}
