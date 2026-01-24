package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double rad) {}
}
