package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double turretPositionRad = 0;
        public double turretVelocityRadPerSec = 0;
        public double turretAppliedVoltage = 0;
        public double[] turretCurrentAmps = new double[]{};
        
        public double hoodPositionRad = 0;
        public double hoodAppliedVolts = 0;

        public double shooterVelocityRpm = 0;
        public double shooterAppliedVolts = 0;
    }
    public default void updateInputs(TurretIOInputs inputs){}

    //Set the turret to an angle using Motion Magic
    public default void setTurretPosition(Rotation2d position){}
    //Set the hood to an angle using Motion Magic (Might be over kill, so it needs to be tested)
    public default void setHoodPosition(Rotation2d position){}
    //Set the velocity for the flywheel for consistency and precision
    public default void setShooterVelocity(double rmp){}
    // just stops all the motors
    public default void stop(){}


}

