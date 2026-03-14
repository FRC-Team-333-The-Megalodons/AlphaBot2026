package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOKraken implements SpindexerIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;

  public SpindexerIOKraken() {
    CANBus rio = CANBus.roboRIO();
    motor = new TalonFX(SpindexerConstants.MOTOR_ID, rio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = SpindexerConstants.GEAR_RATIO;
    config.Slot0.kP = SpindexerConstants.kP;
    config.Slot0.kA = SpindexerConstants.kA;
    config.Slot0.kV = SpindexerConstants.kV;

    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocitySignal, voltageSignal, currentSignal);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocitySignal, voltageSignal, currentSignal);

    inputs.velocityRps = velocitySignal.getValueAsDouble();
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentAmps = currentSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
    Logger.recordOutput("Spindexer/Voltage", volts);
  }
}
