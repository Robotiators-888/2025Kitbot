// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  public SparkMax leftLeader;
  public SparkMax leftFollower;
  public SparkMax rightLeader;
  public SparkMax rightFollower;

  public RelativeEncoder leftLeaderEncoder;
  public RelativeEncoder rightLeaderEncoder;
  public RelativeEncoder leftFollowerEncoder;
  public RelativeEncoder rightFollowerEncoder;

  public DifferentialDrivePoseEstimator m_poseEstimator;
  private Pose2d odometryPose = new Pose2d();

  DifferentialDriveOdometry driveOdometry;

  private final DifferentialDrive drive;
  private static AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);


  public DriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless); 

    leftLeaderEncoder = leftLeader.getEncoder();
    rightLeaderEncoder = rightLeader.getEncoder();
    leftFollowerEncoder = leftFollower.getEncoder();
    rightFollowerEncoder = rightFollower.getEncoder();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    config.inverted(true);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.inverted(false);
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_poseEstimator =
    new DifferentialDrivePoseEstimator(Constants.DriveConstants.KDriveKinematics,
    navx.getRotation2d(),
    leftLeaderEncoder.getPosition(),
    rightLeaderEncoder.getPosition(),
        new Pose2d(0, 0, new Rotation2d(0)));
  }


  @Override
  public void periodic() {}


  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(Math.pow(xSpeed, 2), Math.pow(zRotation,2));
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed,
      DoubleSupplier zRotation) {
    return Commands.run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()),
        driveSubsystem);
  }

   public void setPosition(Pose2d position) {
    //driveOdometry.resetPosition(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()), this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
    //new Pose2d(0, 0, new Rotation2d()));
     //zeroEncoders();
     driveOdometry.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(), position);
   }
        
public void resetPose(Pose2d pose) {
    //zeroEncoders();
    //driveOdometry.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(),
      //  pose);
    m_poseEstimator.resetPosition(navx.getRotation2d(), new DifferentialDriveWheelPositions(0, 0), pose);
    this.odometryPose = pose;
    driveOdometry.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(), odometryPose);
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public ChassisSpeeds getChassisSpeeds() {  
    double rSpeedRPM = rightLeaderEncoder.getVelocity();
    double lSpeedRPM = leftLeaderEncoder.getVelocity();
    double rSpeedMPS = rSpeedRPM*Units.inchesToMeters(Constants.DriveConstants.wheelDiameterIN)*Math.PI;
    double lSpeedMPS = lSpeedRPM*Units.inchesToMeters(Constants.DriveConstants.wheelDiameterIN)*Math.PI;
    return Constants.DriveConstants.KDriveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(lSpeedMPS, rSpeedMPS));
  }  

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    //TODO:Find uses for TargetSpeeds, driveRobotRelative is needed.
  }




 private static DriveSubsystem INSTANCE = null;
  public static DriveSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new DriveSubsystem();
    }
    return INSTANCE;
  }
}

