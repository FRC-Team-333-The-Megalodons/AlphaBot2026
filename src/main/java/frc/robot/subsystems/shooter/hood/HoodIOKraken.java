package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class HoodIOKraken implements HoodIO {
  private final TalonFX motor = new TalonFX(HoodConstants.MOTOR_ID);

  public HoodIOKraken() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
    config.Slot0.kP = HoodConstants.kP;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setPosition(double rad) {
    motor.setControl(new PositionVoltage(Units.radiansToRotations(rad)));
  }
}
