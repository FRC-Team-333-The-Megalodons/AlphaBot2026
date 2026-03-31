package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOKraken implements ClimberIO {

  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID, rio);
  private final CANdi candi = new CANdi(ClimberConstants.CANDI_ID, rio);
  private final DigitalInput magneticSensor =
      new DigitalInput(ClimberConstants.LIMIT_SWITCH_CHANNEL);

  private final StatusSignal<S1StateValue> s1State;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // Tracks whether we have zeroed at least once since robot boot.
  // We refuse to run Motion Magic until this is true.
  private boolean hasZeroed = false;

  // Tracks whether soft limits have been applied after zeroing.
  // We only need to apply them once — re-applying on every zero
  // is harmless but wastes CAN bandwidth.
  private boolean softLimitsEnabled = false;

  // Tracks the previous limit switch state so we only zero on the
  // rising edge (switch goes from false to true), not every loop
  // while it stays triggered.
  private boolean lastLimitSwitch = false;

  public ClimberIOKraken() {
    var candiConfig = new CANdiConfiguration();

    // Rev Magnetic Limit switch is Active Low.
    candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;

    candi.getConfigurator().apply(candiConfig);

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // SensorToMechanismRatio means all position and velocity signals
    // come back in mechanism rotations (after the 45:1 gearbox),
    // not motor rotations. This is what we want.
    config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    config.Slot0.kP = ClimberConstants.kP;
    config.Slot0.kI = ClimberConstants.kI;
    config.Slot0.kD = ClimberConstants.kD;
    config.Slot0.kS = ClimberConstants.kS;
    config.Slot0.kV = ClimberConstants.kV;
    config.Slot0.kA = ClimberConstants.kA;
    config.Slot0.kG = ClimberConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.kCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ClimberConstants.kAcceleration;
    config.MotionMagic.MotionMagicJerk = ClimberConstants.kJerk;

    // Soft limits are NOT enabled at boot. The motor position is garbage
    // until we zero against the limit switch. Enabling soft limits with
    // a random position would either lock the motor out or provide no
    // protection. They are enabled dynamically in zeroPosition().

    config.CurrentLimits.StatorCurrentLimit = 70.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    supplyCurrentAmps = motor.getSupplyCurrent();
    s1State = candi.getS1State();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, currentAmps, supplyCurrentAmps, s1State);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, supplyCurrentAmps);

    // Magnetic limit switches are normally-closed so the DigitalInput
    // reads false when triggered. Invert here so limitSwitchTriggered
    // is true when the magnet is actually at the bottom.
    boolean limitTriggered = (s1State.getValue() == S1StateValue.Low);

    if (limitTriggered && !lastLimitSwitch) {
      zeroPosition();
    }
    lastLimitSwitch = limitTriggered;

    inputs.positionRot = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.supplyAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.limitSwitchTriggered = limitTriggered;
    inputs.hasZeroed = hasZeroed;
    inputs.rawPosition = motor.get();
  }

  @Override
  public boolean isAttached() {
    return magneticSensor.get();
  }

  @Override
  public void moveTo(double positionRot) {
    // Block Motion Magic until we have a valid zero
    if (!hasZeroed) {
      stop();
      return;
    }
    motor.setControl(motionMagicRequest.withPosition(positionRot));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setDutyCycle(double percent) {
    motor.setControl(dutyCycleRequest.withOutput(percent));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void zeroPosition() {
    motor.setPosition(0.0);
    hasZeroed = true;

    // Enable soft limits now that 0.0 is a known-good reference.
    // Only apply once to avoid burning CAN bandwidth on every re-zero.
    if (!softLimitsEnabled) {
      enableSoftLimits();
    }
  }

  /**
   * Dynamically applies software limits using the Phoenix 6 config group API. This only touches the
   * SoftwareLimitSwitch config — all other motor config is untouched.
   */
  private void enableSoftLimits() {
    var softLimits = new SoftwareLimitSwitchConfigs();

    // Forward = positive motor direction = extending = MAX
    softLimits.ForwardSoftLimitThreshold = ClimberConstants.kMaxPositionRot;
    softLimits.ForwardSoftLimitEnable = true;

    // Reverse = negative motor direction = retracting past zero = MIN
    softLimits.ReverseSoftLimitThreshold = ClimberConstants.kMinPositionRot;
    softLimits.ReverseSoftLimitEnable = true;

    motor.getConfigurator().apply(softLimits);
    softLimitsEnabled = true;

    System.out.println(
        "[Climber] Soft limits enabled: reverse="
            + ClimberConstants.kMinPositionRot
            + " forward="
            + ClimberConstants.kMaxPositionRot);
  }

  @Override
  public boolean atTarget(double positionRot) {
    double currentPos = position.getValueAsDouble();
    double currentVel = velocity.getValueAsDouble();

    boolean atPosition =
        Math.abs(positionRot - currentPos) < ClimberConstants.POSITION_TOLERANCE_ROT;
    boolean notMoving = Math.abs(currentVel) < ClimberConstants.VELOCITY_TOLERANCE_RPS;

    return atPosition && notMoving;
  }
}
