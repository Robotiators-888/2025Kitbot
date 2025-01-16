// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {

  TalonSRX rollerMotor;
  double Roller_Trigger_Speed = 0.5;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new TalonSRX(RollerConstants.ROLLER_MOTOR_ID);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    rollerMotor.configPeakCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {}

  public void spinRoller(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void rollerChange(double changeamount) {
    if (changeamount < 0){
      Roller_Trigger_Speed = Math.max(0, Roller_Trigger_Speed - changeamount);
    }
    if (changeamount > 0)
    Roller_Trigger_Speed = Math.min(0.5, Roller_Trigger_Speed + changeamount);
    SmartDashboard.putNumber("Roller Speed", Roller_Trigger_Speed);
  }

  public double rollerScaleFactor() {
    return Roller_Trigger_Speed;
  }

  public void runRoller(double forward, double backward) {
    rollerMotor.set(ControlMode.PercentOutput, forward - backward);
  }
}
