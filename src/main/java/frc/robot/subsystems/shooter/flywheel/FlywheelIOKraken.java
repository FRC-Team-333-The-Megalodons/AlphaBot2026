package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOKraken implements FlywheelIO {
  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, rio);
  private final TalonFX motor2 = new TalonFX(FlywheelConstants.MOTOR_2_ID, rio);

  // Cached status signals — read from motor2 (the leader)
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;

  // Reusable control requests — no per-loop allocations
  private final MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public FlywheelIOKraken() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Slot 0 Gains
    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kA = FlywheelConstants.kA;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;

    config.MotionMagic.MotionMagicCruiseVelocity = FlywheelConstants.MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = FlywheelConstants.MAX_JERK;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.Voltage.PeakForwardVoltage = 9.0;
    // config.Voltage.PeakReverseVoltage = -1.0; // Flywheel should never have to rotate backwards.

    // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
    // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

    // config.CurrentLimits.SupplyCurrentLimit = 45.0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.CurrentLimits.StatorCurrentLimit = 95.0;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Motor 1 follows Motor 2 in the opposite direction (counter-rotation for shooting)
    motor.setControl(new Follower(motor2.getDeviceID(), MotorAlignmentValue.Opposed));

    // Cache status signals from the LEADER motor (motor2)
    velocitySignal = motor2.getVelocity();
    voltageSignal = motor2.getMotorVoltage();
    statorCurrentSignal = motor2.getStatorCurrent();
    supplyCurrentSignal = motor2.getSupplyCurrent();

    // Set update frequencies — only read what we need at the rate we need
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal);

    // Kill all default status frames we don't use. This is the big one(maybe)
    ParentDevice.optimizeBusUtilizationForAll(motor, motor2);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal);

    inputs.velocityRPM = velocitySignal.getValueAsDouble() * 60.0;
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.statorAmps = statorCurrentSignal.getValueAsDouble();
    inputs.supplyAmps = supplyCurrentSignal.getValueAsDouble();
  }

  @Override
  public boolean atTarget(double targetRPM) {
    double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    return Math.abs(targetRPM) > 0
        && Math.abs(Math.abs(currentRPM) - Math.abs(targetRPM))
            < FlywheelConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void moveTo(double rpm) {
    motor2.setControl(mmVelocity.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor2.setControl(voltageRequest.withOutput(volts));
  }
}
