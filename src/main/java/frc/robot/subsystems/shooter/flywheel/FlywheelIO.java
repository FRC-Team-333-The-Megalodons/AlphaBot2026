package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.interfaces.Automatable;

public interface FlywheelIO extends Automatable {
  @AutoLog
  public static class FlywheelIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default double rpsToRPM(AngularVelocity rps) { return rps.in(RotationsPerSecond) * 60.0; }
  public default AngularVelocity rpmToRPS(double rpm) { return RotationsPerSecond.of(rpm/60.0);   }
}
