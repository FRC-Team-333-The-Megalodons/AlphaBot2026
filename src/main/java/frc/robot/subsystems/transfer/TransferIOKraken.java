package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TransferIOKraken implements TransferIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(TransferConstants.MOTOR_ID, rio);

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
