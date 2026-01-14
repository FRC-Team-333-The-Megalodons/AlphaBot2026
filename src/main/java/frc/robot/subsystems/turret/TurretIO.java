package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public class TurretIO {
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
}
