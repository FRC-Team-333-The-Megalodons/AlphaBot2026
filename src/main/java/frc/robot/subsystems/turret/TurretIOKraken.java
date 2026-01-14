package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOKraken implements TurretIO {
    CANBus rioBus = CANBus.roboRIO();
    private final TalonFX turret = new TalonFX(0, rioBus);
    private final TalonFX hood = new TalonFX(0, rioBus);
    private final TalonFX shooter = new TalonFX(0, rioBus);
    
    private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);   
    private final VelocityVoltage shooterRequest = new VelocityVoltage(0);

    public TurretIOKraken(){
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        /*for now I will just assume the 120:1 gear ratio but later needs to be changed
        This is also assuming that we will have the absolute encoder on the output shaft
        TODO: Make sure to change these based on the actual gear ratio*/
        turretConfig.Feedback.SensorToMechanismRatio =  120;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2; //rps
        turretConfig.MotionMagic.MotionMagicAcceleration = 4; //rps^2
        turret.getConfigurator().apply(turretConfig);

    }
}
