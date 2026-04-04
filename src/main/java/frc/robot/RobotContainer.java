// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.energy.BatteryLogger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberIOKrakenSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOKrakenSim;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.LedIO;
import frc.robot.subsystems.leds.LedIOCANdle;
import frc.robot.subsystems.leds.LedIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOKraken;
import frc.robot.subsystems.pivot.PivotIOKrakenSim;
import frc.robot.subsystems.shooter.Targeting.Targeting;
import frc.robot.subsystems.shooter.Targeting.TargetingIO;
import frc.robot.subsystems.shooter.Targeting.TargetingIOReal;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.EnergyLimitMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOKrakenSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOKrakenSim;
import frc.robot.subsystems.shooter.turret.TurretIOYAMS;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOKraken;
import frc.robot.subsystems.spindexer.SpindexerIOKrakenSim;
import frc.robot.subsystems.tracker.RobotStateTracker;
import frc.robot.subsystems.tracker.ShiftTracker;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOKraken;
import frc.robot.subsystems.transfer.TransferIOKrakenSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Energy tracking
  private final BatteryLogger batteryLogger = new BatteryLogger();

  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Transfer transfer;
  private final Flywheel flywheel;
  private final Vision vision;
  private final Targeting targeting;
  private final Pivot pivot;
  private final Turret turret;
  private final Led leds;
  private final Climber climber;
  private final RobotStateTracker stateTracker;
  private final ShiftTracker shiftTracker;

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> sysIdChooser;
  private static final Command NO_SYSID = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    shiftTracker = new ShiftTracker();

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                batteryLogger);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        targeting = new Targeting(new TargetingIOReal(), drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIOKraken(), drive::robotFieldVelocity, batteryLogger);
        spindexer = new Spindexer(new SpindexerIOKraken(), batteryLogger);
        transfer = new Transfer(new TransferIOKraken(), batteryLogger);
        flywheel =
            new Flywheel(new FlywheelIOKraken(), targeting::getTargetDistance, batteryLogger);
        pivot = new Pivot(new PivotIOKraken(), batteryLogger);
        turret =
            new Turret(
                new TurretIOYAMS(),
                targeting::getTargetAngle,
                drive::getRotation,
                targeting::getTargetAngularVelocityRadPerSec,
                drive::getFieldAngularVelocity,
                batteryLogger);
        leds = new Led(new LedIOCANdle(), vision.seesTagsSupplier(0), vision.seesTagsSupplier(1));
        climber = new Climber(new ClimberIOKraken(), batteryLogger);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                batteryLogger);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        targeting = new Targeting(new TargetingIOReal(), drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIOKrakenSim(), drive::robotFieldVelocity, batteryLogger);
        spindexer = new Spindexer(new SpindexerIOKrakenSim(), batteryLogger);
        transfer = new Transfer(new TransferIOKrakenSim(), batteryLogger);
        flywheel =
            new Flywheel(new FlywheelIOKrakenSim(), targeting::getTargetDistance, batteryLogger);
        pivot = new Pivot(new PivotIOKrakenSim(), batteryLogger);
        turret =
            new Turret(
                new TurretIOKrakenSim(),
                targeting::getTargetAngle,
                drive::getRotation,
                targeting::getTargetAngularVelocityRadPerSec,
                drive::getFieldAngularVelocity,
                batteryLogger);
        leds = new Led(new LedIOSim(), vision.seesTagsSupplier(0), vision.seesTagsSupplier(1));
        climber = new Climber(new ClimberIOKrakenSim(), batteryLogger);
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                batteryLogger);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        targeting = new Targeting(new TargetingIO() {}, drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIO() {}, drive::robotFieldVelocity, batteryLogger);
        spindexer = new Spindexer(new SpindexerIO() {}, batteryLogger);
        transfer = new Transfer(new TransferIO() {}, batteryLogger);
        flywheel = new Flywheel(new FlywheelIO() {}, targeting::getTargetDistance, batteryLogger);
        pivot = new Pivot(new PivotIO() {}, batteryLogger);
        turret =
            new Turret(
                new TurretIO() {},
                targeting::getTargetAngle,
                drive::getRotation,
                targeting::getTargetAngularVelocityRadPerSec,
                drive::getFieldAngularVelocity,
                batteryLogger);
        leds = new Led(new LedIO() {}, vision.seesTagsSupplier(0), vision.seesTagsSupplier(1));
        climber = new Climber(new ClimberIO() {}, batteryLogger);
        break;
    }
    drive.seed();
    stateTracker =
        new RobotStateTracker(
            drive::getPose, flywheel::ready, flywheel::isPreSpunUp, intake::getAppliedVolts);

    registerNamedCommands();

    //  Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // SysId chooser (separate dropdown)
    sysIdChooser = new LoggedDashboardChooser<>("SysId Routines");
    sysIdChooser.addDefaultOption("None", NO_SYSID);
    sysIdChooser.addOption("Drive SysId", drive.characterize());
    sysIdChooser.addOption("Drive Wheel Radius", DriveCommands.wheelRadiusCharacterization(drive));
    sysIdChooser.addOption("Drive Simple FF", DriveCommands.feedforwardCharacterization(drive));
    sysIdChooser.addOption("Flywheel SysId", flywheel.characterize());
    sysIdChooser.addOption("Intake SysId", intake.characterize());
    sysIdChooser.addOption("Spindexer SysId", spindexer.characterize());
    sysIdChooser.addOption("Transfer SysId", transfer.characterize());
    sysIdChooser.addOption("Turret SysId", turret.characterize());
    sysIdChooser.addOption("Pivot SysId", pivot.characterize());

    SmartDashboard.putData("Pathfind to Depot", PathfindCommands.pathfindToDepot(drive));
    SmartDashboard.putData("Turret/00 Go to 45", turret.rotateToField(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("Turret/01 Go to 90", turret.rotateToField(Rotation2d.kCW_90deg));
    SmartDashboard.putData(
        "Turret/02 Go to 135", turret.rotateToField(Rotation2d.fromDegrees(135)));
    SmartDashboard.putData("Turret/03 Go to 180", turret.rotateToField(Rotation2d.k180deg));

    SmartDashboard.putData(
        "Turret/04 Go to -45", turret.rotateToField(Rotation2d.fromDegrees(-45)));
    SmartDashboard.putData("Turret/05 Go to -90", turret.rotateToField(Rotation2d.kCCW_90deg));
    SmartDashboard.putData(
        "Turret/06 Go to -135", turret.rotateToField(Rotation2d.fromDegrees(-135)));
    SmartDashboard.putData(
        "Turret/07 Go to -180", turret.rotateToField(Rotation2d.k180deg.unaryMinus()));

    SmartDashboard.putData("Turret/08 Go to 0", turret.rotateToField(Rotation2d.kZero));
    SmartDashboard.putData("Turret/Reseed Abs Position", turret.reseedPosition());

    RobotController.setBrownoutVoltage(Voltage.ofBaseUnits(5.5, Volts));

    seedTurret();
    configureButtonBindings();
  }

  public void periodicAfterScheduler() {
    batteryLogger.setBatteryVoltage(RobotController.getBatteryVoltage());
    batteryLogger.setRioCurrent(RobotController.getInputCurrent());
    batteryLogger.periodicAfterScheduler();
  }

  public void seedTurret() {
    turret.seed();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("DriveToOutpost", PathfindCommands.driveToTheOutpost(drive));
    NamedCommands.registerCommand("PivotDown", pivot.motionMagicDown());
    NamedCommands.registerCommand("PivotDown", pivot.motionMagicDown().withTimeout(2));
    NamedCommands.registerCommand(
        "ShootOnMove", ShootingCommands.shootOnMove(flywheel, turret, spindexer, transfer, pivot));
    NamedCommands.registerCommand("Intake", intake.dynamicIngest());
    NamedCommands.registerCommand("ClimbingPosition", climber.extend());
    NamedCommands.registerCommand("Climb", climber.retract());
    NamedCommands.registerCommand("ClimbZero", climber.zeroEncoder());

    // Teleop climb sequence (pathfind + precision drive, no climber mechanism):
    NamedCommands.registerCommand("ClimbSequence", PathfindCommands.climbSequence(drive));
    NamedCommands.registerCommand(
        "AutoClimb", PathfindCommands.autonomousClimbSequence(drive, climber));
  }

  public void onDriverStationConnected() {
    targeting.seed();
  }

  public void resetTransfer() {
    transfer.stop();
  }

  public void resetFlywheel() {
    flywheel.resetPreSpin();
  }

  private void configureOperatorBindings() {
    // Intake
    operatorController.L1().whileTrue(intake.ingest());
    operatorController.L2().whileTrue(intake.eject());

    // Pivot
    operatorController.R1().whileTrue(pivot.runPercent(() -> 0.1));
    operatorController.R2().whileTrue(pivot.runPercent(() -> -0.15));

    // Spindexer
    operatorController.povRight().whileTrue(spindexer.spin());
    operatorController.povLeft().whileTrue(spindexer.eject());

    // Transfer
    operatorController.cross().whileTrue(transfer.feedShooter());

    // Climber
    operatorController.povDown().whileTrue(climber.driveDown(0.70));
    operatorController.povUp().whileTrue(climber.driveUp(0.70));

    // Flywheel
    operatorController
        .triangle()
        .whileTrue(ShootingCommands.shootOnMove(flywheel, turret, spindexer, transfer, pivot));

    // Override Limits
    operatorController.touchpad().onTrue(flywheel.setEnergyLimits(EnergyLimitMode.UNLIMITED));
    operatorController.touchpad().onFalse(flywheel.setEnergyLimits(EnergyLimitMode.DEFAULT));
  }

  private void configureDriverBindings() {

    driverController.L3().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController
        .R3()
        .whileTrue(
            DriveCommands.faceHubAlternative(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                true));

    driverController
        .triangle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    DriverStation.getAlliance().get() == Alliance.Blue
                        ? Rotation2d.fromDegrees(0)
                        : Rotation2d.fromDegrees(180)));

    driverController
        .cross()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    DriverStation.getAlliance().get() == Alliance.Blue
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0)));

    driverController
        .square()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    DriverStation.getAlliance().get() == Alliance.Blue
                        ? Rotation2d.fromDegrees(90)
                        : Rotation2d.fromDegrees(-90)));

    driverController
        .circle()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    DriverStation.getAlliance().get() == Alliance.Blue
                        ? Rotation2d.fromDegrees(-90)
                        : Rotation2d.fromDegrees(90)));

    driverController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverController.L1().whileTrue(intake.eject());
    driverController.R1().whileTrue(intake.ingest());

    driverController.PS().whileTrue(ShootingCommands.dashboardRPMControl(flywheel));

    driverController
        .povUp()
        .whileTrue(Commands.parallel(spindexer.spin(), transfer.feedShooter(), intake.ingest()));

    driverController.povRight().whileTrue(PathfindCommands.autonomousClimbSequence(drive, climber));
  }

  private void configureDefaultBindings() {
    leds.setDefaultCommand(leds.gameStateAwareLeds(stateTracker));

    flywheel.setDefaultCommand(flywheel.shootOnMoveSpinUp());

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    targeting.setDefaultCommand(targeting.defaultTargetingBehavior());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDefaultBindings();
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // If a SysId routine is selected (anything other than "None"), run that instead of the auto
    Command sysIdCommand = sysIdChooser.get();
    if (sysIdCommand != null && sysIdCommand != NO_SYSID) {
      return sysIdCommand;
    }
    return autoChooser.get();
  }
}
