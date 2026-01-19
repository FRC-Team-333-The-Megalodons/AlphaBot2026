package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TurretIOHardWare implements TurretIO {
  CANBus rioBus = CANBus.roboRIO();
  private final TalonFX turret = new TalonFX(10, rioBus);
  private final TalonFX hood = new TalonFX(11, rioBus);
  private final SparkFlex shooterFlex = new SparkFlex(12, MotorType.kBrushless);

  /*This is Kraken option -> uncomment if needed
  private final TalonFX shooterKraken = new TalonFX(0, rioBus);
  private final VelocityVoltage shooterRequest = new VelocityVoltage(0);
  */

  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);

  public TurretIOHardWare() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 120.0;
    turret.getConfigurator().apply(config);

    SparkFlexConfig flexConfig = new SparkFlexConfig();
    flexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    shooterFlex.configure(
        flexConfig,
        SparkFlex.ResetMode.kResetSafeParameters,
        SparkFlex.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretPositionRad = Units.rotationsToRadians(turret.getPosition().getValueAsDouble());
    inputs.turretVelocityRadPerSec =
        Units.rotationsToRadians(turret.getVelocity().getValueAsDouble());
    inputs.shooterVelocityRpm = shooterFlex.getEncoder().getVelocity();

    /* For Kraken:
    inputs.shooterVelocityRpm = shooterKraken.getVelocity().getValueAsDouble() * 60.0;
    */
  }

  @Override
  public void setShooterVelocity(double rpm) {
    shooterFlex.getClosedLoopController().setReference(rpm, SparkFlex.ControlType.kVelocity);

    /* For Kraken:
    shooterKraken.setControl(shooterRequest.withVelocity(rpm / 60.0));
    */
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
  public void stop() {
    turret.stopMotor();
    hood.stopMotor();
    shooterFlex.stopMotor();
  }
}
