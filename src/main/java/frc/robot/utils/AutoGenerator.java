package frc.robot.utils;

import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;

public class AutoGenerator extends SubsystemBase {
  // public ChassisSpeeds java.util.function.Supplier.get()
  public static DriveSubsystem Drivetrain = DriveSubsystem.getInstance();
  private static AutoGenerator INSTANCE = null;

  public AutoGenerator() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
        Drivetrain::getPose, // Robot pose supplier
        Drivetrain::resetPose, // Method to reset odometry (will be called if your auto has starting pose)
        Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> Drivetrain.driveRobotRelative(speeds),
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            Drivetrain // Reference to this subsystem to set requirements
        );

        registerAllCommands();
  }

  public void registerAllCommands() {}

  public static AutoGenerator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AutoGenerator();
    }

    return INSTANCE;
  }
}
