package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionRot = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double statorAmps = 0.0;
    public double supplyAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean hasZeroed = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void moveTo(double positionRot) {}

  public default void setVoltage(double volts) {}

  public default void setDutyCycle(double percent) {}

  public default void stop() {}

  public default void zeroPosition() {}

  public default boolean atTarget(double positionRot) {
    return false;
  }
}
