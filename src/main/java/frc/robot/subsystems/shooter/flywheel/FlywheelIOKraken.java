package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class FlywheelIOKraken implements FlywheelIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, rio);

  public FlywheelIOKraken() {
    var config = new TalonFXConfiguration();
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVelocity(double radPerSec) {
    motor.setControl(new VelocityVoltage(Units.radiansToRotations(radPerSec)));
  }
}
