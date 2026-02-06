package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double turretPositionRad = 0.0;
    public double turretVelocityRadPerSec = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;

    public double encoder17Rotations = 0.0;
    public double encoder18Rotations = 0.0;
    public double calculatedAbsPositionRot = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretPosition(Rotation2d position) {}

  public default void setTurretVoltage(double volts) {}

  public default void stop() {}
}
