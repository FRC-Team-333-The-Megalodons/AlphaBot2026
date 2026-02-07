package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class SpindexerIOKraken implements SpindexerIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(SpindexerConstants.MOTOR_ID, rio);

  public SpindexerIOKraken() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = SpindexerConstants.GEAR_RATIO;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.velocityRps = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(new VelocityVoltage(Units.radiansToRotations(rps)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
