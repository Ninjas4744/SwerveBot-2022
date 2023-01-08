// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  private final limeLight cam = new limeLight();
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  // private final JoystickButton zeroGyro =
  //     new JoystickButton(driver, PS4Controller.Button.kCross);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, 1);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY()*0.2,
            () -> -driver.getLeftX()*0.2,
            () -> -driver.getRightX()*0.2,
            () -> robotCentric.get()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Actions that we want to do when the robot is disabled. */
  public void disabledActions() {
    s_Swerve.zeroGyro();
    s_Swerve.resetModuleZeros();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // robotCentric.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    s_Swerve.zeroGyro();
    s_Swerve.resetModuleZeros();
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
PathPlannerTrajectory examplePath = PathPlanner.loadPath("New New Path", new PathConstraints(3,3
  // Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//  Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
 ));

// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
// Or the path can be sampled at a given point in time for custom path following

// Sample the state of the path at 1.2 seconds
PathPlannerState exampleState = (PathPlannerState) examplePath.sample(0);
// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
s_Swerve.resetOdometry(exampleState.poseMeters);
return new PPSwerveControllerCommand(
            examplePath, 
            s_Swerve::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. T une these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            s_Swerve::setModuleStates, // Module states consumer
            s_Swerve // Requires this drive subsystem
);

  }
}
