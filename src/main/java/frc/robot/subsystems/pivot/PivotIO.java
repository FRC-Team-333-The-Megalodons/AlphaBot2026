package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPosition(double rad) {}

  public default void setVoltage(double volts) {}
}
