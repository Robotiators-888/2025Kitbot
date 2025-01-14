// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANRollerSubsystem;

public class Spinny_Auto extends SequentialCommandGroup {

  public Spinny_Auto(){
    DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
    CANRollerSubsystem rollerSubsystem = RobotContainer.rollerSubsystem;
    Command spinRoller = Commands.run(() -> rollerSubsystem.spinRoller(0.5),rollerSubsystem);
    Command stopRoller = Commands.run(() -> rollerSubsystem.spinRoller(0), rollerSubsystem);
    Command arcadeDrive = Commands.run(() -> driveSubsystem.arcadeDrive(0,0.25), driveSubsystem);
    Command stopArcadeDrive = Commands.run(() -> driveSubsystem.arcadeDrive(0,0), driveSubsystem); 

    addCommands(
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        arcadeDrive.withTimeout(6),
        stopArcadeDrive
      ),
      new SequentialCommandGroup(
        spinRoller.withTimeout(5),
        stopRoller
      )
    ));
}}


// Example autonomous command which drives forward for 1 second.
//  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem) {
//    return driveSubsystem.driveArcade(driveSubsystem, () -> 1, () -> -0.5).withTimeout(5.0);

//}
