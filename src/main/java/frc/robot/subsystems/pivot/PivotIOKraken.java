package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RobotMetrics;

public class PivotIOKraken implements PivotIO {

  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(PivotConstants.MOTOR_ID, rio);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public PivotIOKraken() {
    var config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;
    // config.CurrentLimits.StatorCurrentLimit = 40.0;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kP = PivotConstants.kP;
    config.Slot0.kI = PivotConstants.kI;
    config.Slot0.kD = PivotConstants.kD;
    config.Slot0.kS = PivotConstants.kS;
    config.Slot0.kV = PivotConstants.kV;
    config.Slot0.kA = PivotConstants.kA;

    config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.kCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = PivotConstants.kAcceleration;
    config.MotionMagic.MotionMagicJerk = PivotConstants.kJerk;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(PivotConstants.kMaxAngleDeg);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(PivotConstants.kMinAngleDeg);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    RobotMetrics.start("PivotKrakenUpdateInputs");

    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps);

    inputs.positionDeg = getPositionDeg();
    inputs.velocityRPM = velocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = getAppliedVoltage();
    inputs.currentAmps = currentAmps.getValueAsDouble();

    RobotMetrics.stop("PivotKrakenUpdateInputs");
  }

  @Override
  public boolean atTarget(double degrees) {
    double currentDeg = Units.rotationsToDegrees(position.getValueAsDouble());
    double currentRPM = velocity.getValueAsDouble() * 60.0;

    boolean atPosition = Math.abs(degrees - currentDeg) < PivotConstants.POSITION_TOLERANCE_DEG;
    boolean notMoving = Math.abs(currentRPM) < PivotConstants.VELOCITY_TOLERANCE_RPM;

    return atPosition && notMoving;
  }

  @Override
  public double getAppliedVoltage() {
    return appliedVolts.getValueAsDouble();
  }

  @Override
  public double getPositionDeg() {
    return Units.rotationsToDegrees(position.getValueAsDouble());
  }

  @Override
  public void moveTo(double degrees) {
    motor.setControl(positionRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void motionMagicTo(double degrees) {
    motor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void set(double input) {
    motor.set(input);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}