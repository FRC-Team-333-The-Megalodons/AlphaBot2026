package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.interfaces.Automatable;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO extends Automatable {
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
  }

  public static final InterpolatingDoubleTreeMap distanceToRPM = buildMap();

  private static InterpolatingDoubleTreeMap buildMap() {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(1.57, 2100.0);
    map.put(1.7, 2180.0);
    map.put(1.9, 2220.0);
    map.put(2.1, 2245.0);
    map.put(2.3, 2280.0);
    map.put(2.67, 2300.0);
    map.put(2.82, 2350.0);
    map.put(3.15, 2420.0);
    map.put(3.5, 2530.0);
    map.put(3.7, 2610.0);
    map.put(4.0, 2660.0);
    map.put(4.2, 2850.0);
    map.put(4.4, 3050.0);
    return map;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default double getRPMFromDistance(double distanceMeters) {
    return distanceToRPM.get(distanceMeters);
  }

  public default double getRPMFromDistance(Distance distance) {
    return distanceToRPM.get(distance.in(Meters));
  }

  public default double rpsToRPM(AngularVelocity rps) {
    return rps.in(RotationsPerSecond) * 60.0;
  }

  public default AngularVelocity rpmToRPS(double rpm) {
    return RotationsPerSecond.of(rpm / 60.0);
  }
}
