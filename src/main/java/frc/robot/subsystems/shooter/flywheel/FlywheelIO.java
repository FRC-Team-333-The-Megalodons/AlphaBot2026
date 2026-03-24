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
    map.put(1.18, 2100.0);
    map.put(1.5, 2135.0);
    map.put(1.7, 2155.0);
    map.put(1.9, 2170.0);
    map.put(2.1, 2190.0);
    map.put(2.333, 2228.0);
    map.put(2.5, 2235.0);
    map.put(2.65, 2255.0);
    map.put(2.8, 2265.0);
    map.put(3.0, 2275.0);
    map.put(3.2, 2290.0);
    map.put(3.45, 2315.0);
    map.put(3.65, 2235.0);
    map.put(3.8, 2600.0);
    map.put(4.1, 2648.0);
    map.put(4.44, 2780.0);
    map.put(4.8, 2795.0);
    map.put(5.0, 2818.0);
    map.put(5.2, 2830.0);
    map.put(5.4, 2845.0);
    map.put(5.5, 2860.0);
    map.put(5.7, 2880.0);
    map.put(6.2, 2920.0);
    map.put(6.4, 2935.0);
    map.put(6.7, 2960.0);
    map.put(7.0, 2980.0);

    // map.put(1.18, 2100.0);
    // map.put(1.3, 2120.0);
    // map.put(1.57, 2140.0);
    // map.put(1.7, 2220.0);
    // map.put(1.9, 2250.0);
    // map.put(2.1, 2270.0);
    // map.put(2.3, 2325.0);
    // map.put(2.5, 2360.0);
    // map.put(2.67, 2370.0);
    // map.put(2.82, 2380.0);
    // map.put(3.15, 2450.0);
    // map.put(3.3, 2480.0);
    // map.put(3.5, 2560.0);
    // map.put(3.7, 2635.0);
    // map.put(3.8, 2660.0);
    // map.put(4.0, 2690.0);
    // map.put(4.2, 2820.0);
    // map.put(4.4, 2840.0);
    // map.put(4.6, 2845.0);
    // map.put(4.8, 2850.0);
    // map.put(5.0, 2860.0);
    // map.put(5.2, 2860.0);
    // map.put(5.4, 2880.0);
    // map.put(5.5, 2895.0);
    // map.put(6.2, 2920.0);
    // map.put(6.4, 2935.0);
    // map.put(6.7, 2960.0);
    // map.put(7.0, 2980.0);

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
