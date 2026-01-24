package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX motor = new TalonFX(ShooterConstants.MOTOR_ID);

  public ShooterIOKraken() {
    var config = new TalonFXConfiguration();
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kP = ShooterConstants.kP;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVelocity(double radPerSec) {
    motor.setControl(new VelocityVoltage(Units.radiansToRotations(radPerSec)));
  }
}
