// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 20;
    public static final int LEFT_FOLLOWER_ID = 21;
    public static final int RIGHT_LEADER_ID = 22;
    public static final int RIGHT_FOLLOWER_ID = 23;
    public static final double GEARRATIO = 10.86; //gear ratio from output shaft of motor to wheel axle
    
    public static final class Autonomous {
      public static final double kmaxAcceleration = 0.2;
      public static final double kDriveSpeed = 0.75; 
      public static final double FreeSpeedMaxMotor = 5700; // sparkmaxconfig has a max rpm of 5700 rpm on rev robotics 
      //RPM to M/: -> RPM / 60 , then multiply by the diameter TODO: convert truemaxspeed to m/s, shouldn't be above 10
      // neo brushless free RPM: 5676, 
        // public static final double kmaxDriveSpeed = #.#; // driving underload in meters/second ^2, (Not max velocity, encodes motor torque)
        // with no testing, can find by taking 85% of the no load speed
      //TODO May need Moment of inertia for the robot
    
    }


    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
     //chasse config
    public static final double kTrackWidth = Units.inchesToMeters(25);
    public static final double kWheelBase = Units.inchesToMeters(30);
   

    public static final DifferentialDriveKinematics KDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeedMetersPerSecond = 0.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;  
    public static final double kDrivingMotorFreeSpeedRps = Autonomous.FreeSpeedMaxMotor / 60;// get 95 rpm

    public static final double TrueMaxSpeed = 4845;// this is 85% of the free speed of the motors 
    // could be 5700(.85) = 4845, or divide by 60 then .85 
    // TODO: get correct true max speed default is 5.1 m/s, if this nummber works you can leave it
    //  find friction coefficent, can be set low to decrease robot speed 

    // public int getSmartCurrentRPMLimit();  may be needed 
    public static double wheelDiameterIN = 6;
    public static final double ConversionFactor = Units.inchesToMeters(wheelDiameterIN * Math.PI / GEARRATIO);
    // circumferance multiply by gearratio to find how far the robot has travelled. then converted to meters 
    //convertion over or convertion multiply
 
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 40;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }


}
