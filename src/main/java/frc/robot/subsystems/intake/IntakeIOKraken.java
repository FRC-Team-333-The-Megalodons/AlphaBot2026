package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOKraken implements IntakeIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID, rio);

  public IntakeIOKraken() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);

    // Make sure intake revving up doesn't cause a voltage sag.
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    motor.getConfigurator().apply(config);
  }

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
