// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.controls.Controls;
import frc.robot.subsystems.controls.ControlsIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import java.util.Optional;

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
  private final Controls controls;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

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

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
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
        break;
    }

    // Going to have joysticks whether real or simulated.
    controls = new Controls(new ControlsIOReal());

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

    // Configure the button bindings
    configureTestBindings();
    configureButtonBindings();
  }

  private void configureTestBindings() {
    
    // Configure binds for test mode
    controls.createScheme("test").addBinds(
        (eventLoop) -> {

          controller.L1(eventLoop).onTrue(Commands.print("test mode"));
          controller.square(eventLoop).whileTrue(controls.useScheme("square", Optional.of("test")));
          controller.cross(eventLoop).onTrue(controls.useScheme("cross"));
          controller.triangle(eventLoop).debounce(1.0).onTrue(controls.useScheme("triangle"));

          /*
          // Default command, normal field-relative drive
          // Do not call .setDefaultCommand(), use this pattern instead.
          new Trigger(eventLoop, () -> drive.getCurrentCommand() == null)
              .onTrue(
                  DriveCommands.joystickDrive(
                      drive,
                      () -> -controller.getLeftY(),
                      () -> -controller.getLeftX(),
                      () -> -controller.getRightX()));

          // Lock to 0° when A button is held
          controller
              .a(eventLoop)
              .whileTrue(
                  DriveCommands.joystickDriveAtAngle(
                      drive,
                      () -> -controller.getLeftY(),
                      () -> -controller.getLeftX(),
                      () -> Rotation2d.kZero));

          // Switch to X pattern when X button is pressed
          controller.x(eventLoop).onTrue(Commands.runOnce(drive::stopWithX, drive));

          // Reset gyro to 0° when B button is pressed
          controller
              .b(eventLoop)
              .onTrue(
                  drive
                      .runOnce(
                          () -> {
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                          })
                      .ignoringDisable(true));
            */

        }).save();
        
        // Assume this scheme is self-cancellable due to whileTrue().
        controls.createScheme("square").addBinds(
            (eventLoop) -> {
                controller.L1(eventLoop).onTrue(Commands.print("square mode"));
            }
        ).save();

        // Assume this scheme requires a button release to revert to test scheme.
        // An explicit "exit bind" needs to be declared.
        controls.createScheme("cross").addBinds(
            (eventLoop) -> {
                controller.L1(eventLoop).onTrue(Commands.print("cross mode"));
                controller.cross(eventLoop).debounce(2.0).onFalse(controls.useScheme("test"));
            }
        );

        // Assume this scheme needs a button toggle to revert to test scheme.
        // An explicit "exit bind" needs to be declared.
        controls.createScheme("triangle").addBinds(
            (eventLoop) -> {
                controller.L1(eventLoop).onTrue(Commands.print("triangle mode"));
                controller.triangle().debounce(1.0).onTrue(controls.useScheme("test"));
            }
        );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
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
