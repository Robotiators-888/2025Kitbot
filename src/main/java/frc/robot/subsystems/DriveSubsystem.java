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
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {

  public StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Odometry", Pose2d.struct).publish();
  public StructPublisher<Pose2d> publisher2 =
      NetworkTableInstance.getDefault().getStructTopic("resetpose", Pose2d.struct).publish();

  public StructPublisher<Pose2d> publisher3 =
      NetworkTableInstance.getDefault().getStructTopic("rightencoder", Pose2d.struct).publish();
  // publish right and left encoders

  public SparkMax leftLeader;
  public SparkMax leftFollower;
  public SparkMax rightLeader;
  public SparkMax rightFollower;

  public RelativeEncoder leftLeaderEncoder;
  public RelativeEncoder rightLeaderEncoder;
  public RelativeEncoder leftFollowerEncoder;
  public RelativeEncoder rightFollowerEncoder;

  public DifferentialDrivePoseEstimator m_poseEstimator;
  // private Pose2d odometryPose = new Pose2d();

  DifferentialDriveOdometry driveOdometry;

  // SmartDashboard.putData(navx); // could be used

  private final DifferentialDrive drive;
  private static AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
  // AHRS.NavXComType.setInputRange(-180,180);// find out how to use this seen set...range with PID

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

    navx.setAngleAdjustment(0);
    SmartDashboard.putData(navx);
    // set up differential drive class
    // public final Field2d m_field = new Field2d();
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
    config.encoder.positionConversionFactor(Constants.DriveConstants.ConversionFactor);

    
    config.follow(leftLeader);
    config.inverted(true);

    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.inverted(true);
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(false);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncoders();
    zeroHeading();


    m_poseEstimator = new DifferentialDrivePoseEstimator(Constants.DriveConstants.KDriveKinematics,
        navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(),
        new Pose2d()); // could do Rotation2d.fromDegrees(getAngle())
    // and do new Pose2d(0, 0, new Rotation2d(0)

  }

  public void resetEncoders() {
    rightLeaderEncoder.setPosition(0);
    rightFollowerEncoder.setPosition(0);
    leftLeaderEncoder.setPosition(0);
    leftFollowerEncoder.setPosition(0);
  }// the followers may not be nessary

  public void zeroHeading() {
    navx.reset();
    navx.isCalibrating();
  }

  public static double getGyroHeading() {
    return navx.getRotation2d().getDegrees();
  }
  // may want to use \/ if nessary
  // public Rotation2d getGyroHeading() {
  // return new Rotation2d(-1 * Math.toRadians(navx.getYaw()));
  // }

  public double getTurnRate() {
    return navx.getRate() * (Constants.DriveConstants.Autonomous.kGyroReversed ? -1.0 : 1.0);
  }// feed forward implicates to here

  @Override
  public void periodic() {
    m_poseEstimator.update(navx.getRotation2d(), leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition());

    publisher.set(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("NAVX Angle", navx.getAngle());
    SmartDashboard.putNumber("rightEncoder", getrightLeaderEncoder());
    SmartDashboard.putNumber("leftEncoder", getleftLeaderEncoder());
    SmartDashboard.putNumber("turnRate", getTurnRate());
    SmartDashboard.putNumber("gyroHeading", getGyroHeading());
  }

  public void sarcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(Math.pow(xSpeed, 2), Math.pow(zRotation, 2));
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed,
      DoubleSupplier zRotation) {
    return Commands.run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()),
        driveSubsystem);
  }

  public double getrightLeaderEncoder() {
    return rightLeaderEncoder.getPosition();
  }

  public double getleftLeaderEncoder() {
    return leftLeaderEncoder.getPosition();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public RelativeEncoder getrightEncoder() {
    return getrightEncoder();
  }

  public RelativeEncoder getleftEncoder() {
    return getleftEncoder();
  }// hopeful uses the right encoders

  public void resetPose(Pose2d pose) {
    SmartDashboard.putBoolean("done?", true);
    m_poseEstimator.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(),
        rightLeaderEncoder.getPosition(), pose);
    // code is telling itself that it is alredy where it is
    publisher2.set(getPose());
    this.pose = pose;
  }

  public NavXComType getgyro() {
    return getgyro();
  }// check if this works

  public ChassisSpeeds getChassisSpeeds() {
    double rSpeedRPM = rightLeaderEncoder.getVelocity();
    double lSpeedRPM = leftLeaderEncoder.getVelocity();

    double rSpeedMPS = (rSpeedRPM / Constants.DriveConstants.GEARRATIO) * ((Math.PI * Units.inchesToMeters(Constants.DriveConstants.wheelDiameterIN)) / 60);
    //                       according to the getrate in 2023 /\.  The math here gets 4.4 meters persecond, when the rpm is 6000
    // what was used -> (rSpeedRPM * Units.inchesToMeters(Constants.DriveConstants.wheelDiameterIN) * Math.PI / 60)/Constants.DriveConstants.GEARRATIO;
    double lSpeedMPS = (lSpeedRPM / Constants.DriveConstants.GEARRATIO) * ((Math.PI * Units.inchesToMeters(Constants.DriveConstants.wheelDiameterIN)) / 60);
    // speedRPM * ((2 * Math.PI * Units.inchesToMeters(Constants.DriveConstants.ConversionFactor)) / 60);
    SmartDashboard.putNumber("LM", lSpeedMPS);
    SmartDashboard.putNumber("RM", rSpeedMPS);
    SmartDashboard.putNumber("LR", lSpeedRPM);
    SmartDashboard.putNumber("RR", rSpeedRPM);
    return Constants.DriveConstants.KDriveKinematics
        .toChassisSpeeds(new DifferentialDriveWheelSpeeds(lSpeedMPS, rSpeedMPS));
    // ChassisSpeeds to WheeleSpeeds /\ //TODO find out why the returned speed doesn't change anything

  }

  // public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  //   double leftSpeedMPS = leftLeaderEncoder.getVelocity() * 1/Constants.DriveConstants.GEARRATIO * Constants.DriveConstants.wheelDiameterIN;
  //   double rightSpeedMPS = rightLeaderEncoder.getVelocity() * 1/Constants.DriveConstants.GEARRATIO * Constants.DriveConstants.wheelDiameterIN;
  //   return new DifferentialDriveWheelSpeeds(leftSpeedMPS, rightSpeedMPS);
  // }  // may want to use

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
  driveRobotRelative(
  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); // has 2 meters persecond as
                                                                   // velocity

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(2.0, 2.0);
    // Convert to chassis speeds.
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    // Linear velocity
    double linearVelocity = 0.25;// chassisSpeeds.vxMetersPerSecond;
    // Angular velocity
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    drive.arcadeDrive(linearVelocity, (angularVelocity / 2 * Math.PI));
      //(robotRelativeSpeeds.vxMetersPerSecond / 5), -(robotRelativeSpeeds.omegaRadiansPerSecond / 2 * Math.PI));
  }// angular velocity is mesured in radians persecond, used for omegaradianspersecond.
  //track radius

  private static DriveSubsystem INSTANCE = null;

  public static DriveSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DriveSubsystem();
    }
    return INSTANCE;
  }
}

