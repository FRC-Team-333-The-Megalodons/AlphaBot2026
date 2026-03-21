// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindCommands;
import frc.robot.commands.ShootingCommands;
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

  // Controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));
        targeting = new Targeting(new TargetingIOReal(), drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIOKraken());
        spindexer = new Spindexer(new SpindexerIOKraken());
        transfer = new Transfer(new TransferIOKraken());
        flywheel = new Flywheel(new FlywheelIOKraken(), targeting::getTargetDistance);
        pivot = new Pivot(new PivotIOKraken());
        turret = new Turret(new TurretIOYAMS(), targeting::getTargetAngle, drive::getRotation);
        leds = new Led(new LedIOCANdle());
        climber = new Climber(new ClimberIOKraken());

        // Note:

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS driverController connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera0, drive::getPose));
        targeting = new Targeting(new TargetingIOReal(), drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIOKrakenSim());
        spindexer = new Spindexer(new SpindexerIOKrakenSim());
        transfer = new Transfer(new TransferIOKrakenSim());
        flywheel = new Flywheel(new FlywheelIOKrakenSim(), targeting::getTargetDistance);
        pivot = new Pivot(new PivotIOKrakenSim());
        turret = new Turret(new TurretIOKrakenSim(), targeting::getTargetAngle, drive::getRotation);
        leds = new Led(new LedIOSim());
        climber = new Climber(new ClimberIOKrakenSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        targeting = new Targeting(new TargetingIO() {}, drive::getPose, drive::robotFieldVelocity);
        intake = new Intake(new IntakeIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        transfer = new Transfer(new TransferIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, targeting::getTargetDistance);
        pivot = new Pivot(new PivotIO() {});
        turret = new Turret(new TurretIO() {}, targeting::getTargetAngle, drive::getRotation);
        leds = new Led(new LedIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }
    drive.seed();
    stateTracker =
        new RobotStateTracker(
            drive::getPose, flywheel::ready, flywheel::isPreSpunUp, intake::getAppliedVolts);

    registerNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Flywheel SysId routine
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Turret SysId routines
    autoChooser.addOption(
        "Turret SysId (Quasistatic Forward)",
        turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turret SysId (Quasistatic Reverse)",
        turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turret SysId (Dynamic Forward)", turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turret SysId (Dynamic Reverse)", turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Transfer SysId (Quasistatic Forward)",
        transfer.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Transfer SysId (Quasistatic Reverse)",
        transfer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Transfer SysId (Dynamic Forward)", transfer.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Transfer SysId (Dynamic Reverse)", transfer.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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

    seedTurret();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void seedTurret() {
    turret.seed();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("DriveToOutpost", PathfindCommands.driveToTheOutpost(drive));
    NamedCommands.registerCommand("ClimbSequence", PathfindCommands.climbSequence(drive));
    NamedCommands.registerCommand("ClimbingPosition", climber.extend());
    NamedCommands.registerCommand("Climb", climber.retract());
    NamedCommands.registerCommand(
        "ShootOnMove",
        ShootingCommands.shootOnMove(flywheel, turret, spindexer, transfer, intake, pivot, drive));
    NamedCommands.registerCommand(
        "Intake",
        intake.dynamicIngest(
            () -> {
              var fieldVelocity = drive.robotFieldVelocity();
              double absX = Math.abs(fieldVelocity.dx);
              double absY = Math.abs(fieldVelocity.dy);
              return Math.max(absX, absY);
            }));
  }

  public void onDriverStationConnected() {
    targeting.seed();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // pivot.setDefaultCommand(pivot.runPercent(() -> -driverController.getRightY()));

    leds.setDefaultCommand(leds.gameStateAwareLeds(stateTracker));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    targeting.setDefaultCommand(targeting.defaultTargetingBehavior());

    // pivot.setDefaultCommand(pivot.coordinatedPivot(flywheel, intake));

    driverController
        .L2()
        .whileTrue(
            intake.dynamicIngest(
                () -> {
                  var fieldVelocity = drive.robotFieldVelocity();
                  double absX = Math.abs(fieldVelocity.dx);
                  double absY = Math.abs(fieldVelocity.dy);
                  return Math.max(absX, absY);
                }));

    // Locks wheels in x shape
    driverController.R3().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController
        .R2()
        .whileTrue(
            ShootingCommands.shootOnMove(
                flywheel, turret, spindexer, transfer, intake, pivot, drive));

    // controller.R1().whileTrue(pivot.motionMagicUp());

    driverController
        .L3()
        .whileTrue(
            DriveCommands.faceHubAlternative(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                true /*Face Backwards, because the Turret can only face left/back/right */));

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

    driverController.PS().whileTrue(ShootingCommands.dashboardRPMControl(flywheel));
    driverController.R1().whileTrue(pivot.motionMagicDown());
    driverController.L1().whileTrue(pivot.motionMagicUp());
    driverController
        .povUp()
        .whileTrue(Commands.parallel(spindexer.spin(), transfer.feedShooter(), intake.ingest()));

    // driverController.povUp().whileTrue(PathfindCommands.pathfindToDepot(drive));

    driverController.povRight().onTrue(PathfindCommands.climbSequence(drive));
     // Intake
    operatorController.L2().whileTrue(intake.ingest());
    operatorController.L1().whileTrue(intake.eject());

    // Pivot
    operatorController.R1().whileTrue(pivot.runPercent(() -> 0.1)); // forward(downwards)
    operatorController.R2().whileTrue(pivot.runPercent(() -> -0.15)); // backwards(upwards)

    // Spindexxer
    operatorController.povRight().whileTrue(spindexer.spin());
    operatorController.povLeft().whileTrue(spindexer.eject());

    // //Transfer
    operatorController.cross().whileTrue(transfer.feedShooter());

    // //Shooter
    operatorController
        .triangle()
        .whileTrue(Commands.run(() -> flywheel.setRPMDirect(2200), flywheel));

    // climber
    operatorController.povDown().whileTrue(climber.driveUp(0.50)); // climber up
    operatorController.povUp().whileTrue(climber.driveDown(-0.50)); // climber down
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
