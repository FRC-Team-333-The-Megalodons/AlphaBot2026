package frc.robot.subsystems.pivot;

import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO extends Automatable {

  @AutoLog
  public static class PivotIOInputs {
    public double appliedVolts = 0.0;
    public double positionDeg = 0.0;
    public double velocityRPM = 0.0;
    public double statorAmps = 0.0;
    public double supplyAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void moveTo(double degrees) {}

  public default void motionMagicTo(double degrees) {}

  public default double getAppliedVoltage() {
    return 0;
  }

  public default double getPositionDeg() {
    return 0;
  }

  public default void set(double input) {}

  public default void zeroPosition() {}
}
