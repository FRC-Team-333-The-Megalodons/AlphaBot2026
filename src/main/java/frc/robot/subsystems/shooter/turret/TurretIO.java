package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double positionRad = 0.0;
    // public double velocityRadPerSec = 0.0;
    public double limelightImuYaw = 0.0;
    public double appliedVolts = 0.0;
    public double targetPositionRad = 0.0;

    // Vision Inputs
    public boolean limelightHasTarget = false;
    public double limelightTx = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setPosition(double rad) {}

  public default void setVoltage(double volts) {}
}
