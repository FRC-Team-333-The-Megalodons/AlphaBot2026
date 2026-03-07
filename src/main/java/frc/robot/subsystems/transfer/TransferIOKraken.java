package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TransferIOKraken implements TransferIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  public TransferIOKraken() {
    CANBus rio = CANBus.roboRIO();
    motor = new TalonFX(TransferConstants.MOTOR_ID, rio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = TransferConstants.GEAR_RATIO;
    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocitySignal, voltageSignal, currentSignal);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocitySignal, voltageSignal, currentSignal);

    // If velocityRpm complains because it's no longer in the inputs, just use inputs.appliedVolts
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentAmps = currentSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
