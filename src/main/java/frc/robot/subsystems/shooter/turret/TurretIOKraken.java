package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.util.Units;

public class TurretIOKraken implements TurretIO {
 CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID,rio);
  private final CANcoder cancoder = new CANcoder(TurretConstants.CANCODER_ID);

  public TurretIOKraken() {
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = TurretConstants.CANCODER_OFFSET;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cancoder.getConfigurator().apply(cancoderConfig);

    var config = new TalonFXConfiguration();

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = TurretConstants.CANCODER_ID;
    config.Feedback.RotorToSensorRatio = TurretConstants.GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(cancoder.getPosition().getValueAsDouble());
    // inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setPosition(double rad) {
    motor.setControl(new PositionVoltage(Units.radiansToRotations(rad)));
  }
}
