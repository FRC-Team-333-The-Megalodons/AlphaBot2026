package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOKraken implements TurretIO {
  CANBus rioBus = CANBus.roboRIO();
  private final TalonFX turret = new TalonFX(0, rioBus);
  private final TalonFX hood = new TalonFX(0, rioBus);
  private final TalonFX shooter = new TalonFX(0, rioBus);

  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage shooterRequest = new VelocityVoltage(0);

  public TurretIOKraken() {
    // turret config
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    /*for now I will just assume the 120:1 gear ratio but later needs to be changed
    This is also assuming that we will have the absolute encoder on the output shaft
    TODO: Make sure to change these based on the actual gear ratio*/
    turretConfig.Feedback.SensorToMechanismRatio = 120;
    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2; // rps
    turretConfig.MotionMagic.MotionMagicAcceleration = 4; // rps^2
    turret.getConfigurator().apply(turretConfig);

    // hood config
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Feedback.SensorToMechanismRatio = 80.0;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    hoodConfig.MotionMagic.MotionMagicAcceleration = 3.0;
    hood.getConfigurator().apply(hoodConfig);

    // shooter config
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Feedback.SensorToMechanismRatio = 1.0;
    shooter.getConfigurator().apply(shooterConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Turret Inputs
    inputs.turretPositionRad = Units.rotationsToRadians(turret.getPosition().getValueAsDouble());
    inputs.turretVelocityRadPerSec =
        Units.rotationsToRadians(turret.getVelocity().getValueAsDouble());
    inputs.turretAppliedVolts = turret.getMotorVoltage().getValueAsDouble();
    inputs.turretCurrentAmps = new double[] {turret.getStatorCurrent().getValueAsDouble()};

    // Hood Inputs
    inputs.hoodPositionRad = Units.rotationsToRadians(hood.getPosition().getValueAsDouble());
    inputs.hoodAppliedVolts = hood.getMotorVoltage().getValueAsDouble();

    // Shooter Inputs
    // getVelocity() returns Rotations per Second -> multiply by 60 for RPM
    inputs.shooterVelocityRpm = shooter.getVelocity().getValueAsDouble() * 60.0;
    inputs.shooterAppliedVolts = shooter.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setTurretPosition(Rotation2d position) {
    turret.setControl(turretRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setHoodPosition(Rotation2d position) {
    hood.setControl(hoodRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setShooterVelocity(double rpm) {
    // withVelocity expects Rotations per Second
    shooter.setControl(shooterRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void stop() {
    turret.stopMotor();
    hood.stopMotor();
    shooter.stopMotor();
  }
}
