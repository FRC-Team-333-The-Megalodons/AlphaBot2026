package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TransferIOKraken implements TransferIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(TransferConstants.MOTOR_ID, rio);

  public TransferIOKraken() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

    motor.getConfigurator().apply(config);
  }

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
