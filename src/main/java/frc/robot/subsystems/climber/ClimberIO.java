package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double positionRot = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean limitSwitchTriggered = false;
    public boolean hasZeroed = false;
    public double rawPosition = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  // Motion Magic position control —> method for auto and precise movement
  public default void moveTo(double positionRot) {}

  public default void setVoltage(double volts) {}

  public default void setDutyCycle(double percent) {}

  public default void stop() {}

  public default void zeroPosition() {}

  public default boolean atTarget(double positionRot) {
    return false;
  }

  public default boolean isAttached() {
    return false;
  }
}