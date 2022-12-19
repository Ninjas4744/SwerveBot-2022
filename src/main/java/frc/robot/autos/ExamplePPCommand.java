package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
public class ExamplePPCommand extends SequentialCommandGroup{
    Swerve _driveTerrain;
    public ExamplePPCommand(Swerve _driveTerrain) {
        this._driveTerrain = _driveTerrain;
        // TrajectoryConfig config =
        // new TrajectoryConfig(
        //         Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //     .setKinematics(Constants.Swerve.swerveKinematics);
        
// This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path",2,2);

// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
// Or the path can be sampled at a given point in time for custom path following

// Sample the state of the path at 1.2 seconds
PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

// Print the velocity at the sampled time
System.out.println(exampleState.velocityMetersPerSecond);
    addCommands(
        new InstantCommand(() -> _driveTerrain.resetOdometry(examplePath.getInitialPose())),
        followTrajectoryCommand(examplePath));
    }
    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        // // This is just an example event map. It would be better to have a constant, global event map
        // // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new IntakeDown());
         
        return new PPSwerveControllerCommand(
                traj, 
                _driveTerrain::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),// Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints),// Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                _driveTerrain::setModuleStates, // Module states consumer
                this._driveTerrain // Requires this drive subsystem
        );
    }
    
    
}
