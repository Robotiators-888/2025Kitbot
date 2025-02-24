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
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // private Pose2d odometryPose = new Pose2d();

  public DifferentialDrivePoseEstimator m_poseEstimator;

  private final DifferentialDrive drive;
  private static AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  Pose2d pose = new Pose2d();

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

    navx.setAngleAdjustment(0.0);
    navx.resetDisplacement();
    navx.reset();
    zeroEncoders();
    // navx.getGyroFullScaleRangeDPS(); // Could be useful to use
    // navx.setAngleAdjustment(0);
    m_poseEstimator = new DifferentialDrivePoseEstimator(Constants.Autonomous.KDriveKinematics,
        navx.getRotation2d(),
        leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition(),
        new Pose2d(0, 0, new Rotation2d(0)));
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

  }

  @Override
  public void periodic() {
    m_poseEstimator.update(navx.getRotation2d(), leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition());

    // debug values
    // SmartDashboard.putData("NAVX angle", navx.getAngle());
    SmartDashboard.putNumber("leftencoderVelocity", leftLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("rightencoderVelocity", rightLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("leftencoderposition", leftLeaderEncoder.getPosition());
    SmartDashboard.putNumber("rightencoderposition", rightLeaderEncoder.getPosition());
    SmartDashboard.putNumber("GyroHeading", getGyroHeading());
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("yaw", getYaw());
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("roll", getRoll());
    SmartDashboard.putData(navx);

  }

  public void zeroEncoders() {
    leftLeaderEncoder.setPosition(0.0);
    leftFollowerEncoder.setPosition(0.0);
    rightLeaderEncoder.setPosition(0.0);
    rightFollowerEncoder.setPosition(0.0);
  }

  // debug values
  public double getAngle() {
    return navx.getRotation2d().getDegrees();
  }

  public void resetAngle() {
    navx.reset();
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public static double getGyroHeading() {
    return navx.getRotation2d().getDegrees();
  }

  // public RelativeEncoder??

  // may add getRightLeaderEncoder and GetLeftLeaderEncoder
  // debug values

  public void resetPose(Pose2d pose) {
    // zeroEncoders();
    m_poseEstimator.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition(),
        pose);
    this.pose = pose;
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed,
      DoubleSupplier zRotation) {
    return Commands.run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()),
        driveSubsystem);

    // double left = MathUtil.clamp(1.0, -1.0, 1.0);//TODO what does this do
    // leftLeader.set(left);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
    // may or may not be in meters
  }

  public void setPosition(double x, double y, Rotation2d angle) {
    setPosition(new Pose2d(x, y, angle));
    navx.setAngleAdjustment(angle.getDegrees());
    zeroEncoders();
  }

  public void setPosition(Pose2d position) {
    // driveOdometry.resetPosition(getGyroHeading(),
    // this.rotationsToMeters(leftPrimaryEncoder.getPosition()),
    // this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
    // new Pose2d(0, 0, new Rotation2d()));
    // zeroEncoders();
    m_poseEstimator.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition(), position);
  }

  public double getRate(double input) {
    return (input / Constants.DriveConstants.GEARRATIO)
        * ((2 * Math.PI * Units.inchesToMeters(Constants.Autonomous.wheelDiameterIN)) / 60);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.Autonomous.KDriveKinematics.toChassisSpeeds(gWheelSpeeds()); // used for wheelspeeds to chassis
                                                                                  // speeds
    // takes given wheel speeds and converts it to chassis speeds

  }// using chassis speeds

  public DifferentialDriveWheelSpeeds gWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getRate(leftLeaderEncoder.getVelocity()),
        getRate(rightLeaderEncoder.getVelocity()));
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    drive.arcadeDrive((robotRelativeSpeeds.vxMetersPerSecond / 5),
        -(robotRelativeSpeeds.omegaRadiansPerSecond / 2 * Math.PI));
    // might want to try with normal robot relative speeds
  }

  public void ARcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(Math.pow(xSpeed, 2), Math.pow(zRotation, 2));
  }// not used for path planner autos

  private static DriveSubsystem INSTANCE = null;

  public static DriveSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DriveSubsystem();
    }
    return INSTANCE;
  }
}
