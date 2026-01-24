package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.velocityRps = motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
