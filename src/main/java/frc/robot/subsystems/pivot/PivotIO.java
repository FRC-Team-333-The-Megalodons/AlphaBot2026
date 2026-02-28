package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.interfaces.Automatable;

public interface PivotIO extends Automatable {
  @AutoLog
  public static class PivotIOInputs {
    public double appliedVolts = 0.0;
    public double pivotAngle = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
