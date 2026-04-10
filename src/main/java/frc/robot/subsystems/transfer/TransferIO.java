package frc.robot.subsystems.transfer;

import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface TransferIO extends Automatable {
  @AutoLog
  public static class TransferIOInputs {
    public double appliedVolts = 0.0;
    public double statorAmps = 0.0;
    public double supplyAmps = 0.0;
    public double velocityRpm = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(TransferIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void moveTo(double rpm) {}

  public default boolean atTarget(double rpm) {
    return true;
  }

  public default double getCurrentRPM() { return 0.0; }
}
