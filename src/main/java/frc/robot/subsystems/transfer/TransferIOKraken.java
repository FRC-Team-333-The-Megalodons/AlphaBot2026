package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TransferIOKraken implements TransferIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> tempSignal;

  private final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TransferIOKraken() {
    CANBus rio = CANBus.roboRIO();
    motor = new TalonFX(TransferConstants.MOTOR_ID, rio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = TransferConstants.GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = TransferConstants.kS;
    config.Slot0.kV = TransferConstants.kV;
    config.Slot0.kA = TransferConstants.kA;
    config.Slot0.kP = TransferConstants.kP;

    config.MotionMagic.MotionMagicCruiseVelocity = TransferConstants.MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TransferConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = TransferConstants.MAX_JERK;
    config.Voltage.PeakForwardVoltage = 6.0;
    config.Voltage.PeakReverseVoltage = -4.0;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

    config.CurrentLimits.SupplyCurrentLimit = 15.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = 35.0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    tempSignal = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, currentSignal, supplyCurrent, tempSignal);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltageSignal, currentSignal, supplyCurrent, tempSignal);

    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.statorAmps = currentSignal.getValueAsDouble();
    inputs.supplyAmps = supplyCurrent.getValueAsDouble();
    inputs.velocityRpm = velocitySignal.getValueAsDouble() * 60.0;
    inputs.tempCelsius = tempSignal.getValueAsDouble();
  }

  @Override
  public void moveTo(double rpm) {

    double rps = rpm / 60.0;
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public double getCurrentRPM() {
    return velocitySignal.getValueAsDouble() * 60.0;
  }

  @Override
  public boolean atTarget(double rpm) {
    double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    return Math.abs(Math.abs(currentRPM) - Math.abs(rpm)) < TransferConstants.VELOCITY_TOLERANCE;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
