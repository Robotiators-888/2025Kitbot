// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Autonomous.RollerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Spinny_Auto;
// import frc.robot.commands.Autos; <- got mad at this
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();

  // The driver's controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);


  // The autonomous chooser
  public final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("AutoSelector", autoChooser);
    autoChooser.setDefaultOption("Spinny Auto", new Spinny_Auto());
    // autoChooser.addOption("Next Auto", new );

    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    // autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "runRoller" command from the factory with a fixed
    // value ejecting the gamepiece while the button is held
    driverController.a()
        .whileTrue(new RunCommand (() -> rollerSubsystem.spinRoller(RollerConstants.ROLLER_EJECT_VALUE)));

    driverController.x().onTrue(new InstantCommand(()->rollerSubsystem.rollerChange(-0.1), rollerSubsystem));

    driverController.y().onTrue(new InstantCommand(()->rollerSubsystem.rollerChange(0.1), rollerSubsystem));
    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            driveSubsystem, () -> driverController.getLeftY(), () -> driverController.getRightX()));

    // Set the default command for the roller subsystem to the command from the
    // factory with the values provided by the triggers on the operator controller
    
    rollerSubsystem.setDefaultCommand(new RunCommand(() -> rollerSubsystem.runRoller(
      driverController.getRightTriggerAxis() * rollerSubsystem.rollerScaleFactor(), driverController.getLeftTriggerAxis() * rollerSubsystem.rollerScaleFactor() ),rollerSubsystem));
  }
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
  }
  
  
}


