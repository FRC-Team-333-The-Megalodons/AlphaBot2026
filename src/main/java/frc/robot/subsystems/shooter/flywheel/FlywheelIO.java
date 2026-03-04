package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.interfaces.Automatable;

public interface FlywheelIO extends Automatable {
  @AutoLog
  public static class FlywheelIOInputs {
    public double targetRPM = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
  }

  static InterpolatingDoubleTreeMap distanceToRPM = configureMap();
  
  private static InterpolatingDoubleTreeMap configureMap() {
    var map = new InterpolatingDoubleTreeMap();

    distanceToRPM.put(1.57, 2100.0);
    distanceToRPM.put(1.7, 2180.0);
    distanceToRPM.put(1.9, 2220.0);
    distanceToRPM.put(2.1, 2250.0);
    distanceToRPM.put(2.3, 2280.0);
    distanceToRPM.put(2.67, 2300.0);
    distanceToRPM.put(2.82, 2350.0);
    distanceToRPM.put(3.15, 2390.0);
    distanceToRPM.put(3.5, 2530.0);
    distanceToRPM.put(3.7, 2610.0);
    distanceToRPM.put(4.0, 2660.0);
    distanceToRPM.put(4.2, 2850.0);
    distanceToRPM.put(4.4, 3050.0);

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

  public default double rpsToRPM(AngularVelocity rps) { return rps.in(RotationsPerSecond) * 60.0; }

  public default AngularVelocity rpmToRPS(double rpm) { return RotationsPerSecond.of(rpm/60.0);   }
}
