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
    public double supplyAmps = 0.0;
    public double statorAmps = 0.0;
  }

  public static final InterpolatingDoubleTreeMap distanceToRPM = buildMap();

  private static InterpolatingDoubleTreeMap buildMap() {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(1.18, 2100.0);
    map.put(1.3, 2120.0);
    map.put(1.57, 2140.0);
    map.put(1.7, 2220.0);
    map.put(1.9, 2250.0);
    map.put(2.1, 2270.0);
    map.put(2.3, 2325.0);
    map.put(2.67, 2350.0);
    map.put(2.82, 2380.0);
    map.put(3.15, 2450.0);
    map.put(3.3, 2480.0);
    map.put(3.5, 2560.0);
    map.put(3.7, 2635.0);
    map.put(3.8, 2660.0);
    map.put(4.0, 2690.0);
    map.put(4.2, 2820.0);
    map.put(4.4, 2840.0);
    map.put(4.6, 2870.0);
    map.put(4.8, 2875.0);

    map.put(5.0, 2880.0);

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
