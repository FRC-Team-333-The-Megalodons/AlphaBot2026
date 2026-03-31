package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOKraken implements IntakeIO {
  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID, rio);

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;

  private final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public IntakeIOKraken() {
    var config = new TalonFXConfiguration();

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
    // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

    // config.Voltage.PeakForwardVoltage = 6.0;
    // config.Voltage.PeakReverseVoltage = -6.0;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 15.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = IntakeConstants.kS;
    config.Slot0.kV = IntakeConstants.kV;
    config.Slot0.kA = IntakeConstants.kA;
    config.Slot0.kP = IntakeConstants.kP;

    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = IntakeConstants.MAX_JERK;

    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal);

    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.velocityRpm = velocitySignal.getValueAsDouble() * 60.0;
    inputs.supplyAmps = supplyCurrentSignal.getValueAsDouble();
  }

  @Override
  public void moveTo(double rpm) {
    double rps = rpm / 60.0;
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public boolean atTarget(double rpm) {
    double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    return Math.abs(Math.abs(currentRPM) - Math.abs(rpm)) < IntakeConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
