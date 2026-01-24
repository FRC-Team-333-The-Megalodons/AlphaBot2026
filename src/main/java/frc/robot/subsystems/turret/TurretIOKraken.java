package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class TurretIOKraken implements TurretIO {
  private final TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID);

  public TurretIOKraken() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setPosition(double rad) {
    motor.setControl(new PositionVoltage(Units.radiansToRotations(rad)));
  }
}
