package frc.robot.subsystems.intake;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends Automatable {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts = 0.0;
    public double statorAmps = 0.0;
    public double supplyAmps = 0.0;
    public double velocityRpm = 0.0;
    public double tempCelsius = 0.0;
  }

  /** TODO: Create a speed-to-RPM map once intake PID gains are tuned. */
  public static final InterpolatingDoubleTreeMap speedToVolts = buildMap();

  private static InterpolatingDoubleTreeMap buildMap() {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(0.2, 7.2);
    map.put(0.4, 7.0);
    map.put(0.5, 6.8);
    map.put(0.7, 6.5);
    map.put(0.9, 6.2);
    // map.put(1.1, 5.6);
    // map.put(1.5, 5.5);
    // map.put(1.8, 5.35);
    // map.put(2.0, 5.3);
    // map.put(2.5, 5.2);
    // map.put(5.0, 5.0);
    return map;
  }

  public default double getVoltageFromSpeed(double robotSpeedMetersPerSecond) {
    return speedToVolts.get(robotSpeedMetersPerSecond);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  @Override
  public default void moveTo(double rpm) {}

  @Override
  public default boolean atTarget(double rpm) {
    return true;
  }

  public default boolean isStuck() {
    return false;
  }
}
