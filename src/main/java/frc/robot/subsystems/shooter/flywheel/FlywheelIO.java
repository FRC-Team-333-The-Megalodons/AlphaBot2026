package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocity(double radPerSec) {}

  public default void setVoltage(double volts) {}
}
