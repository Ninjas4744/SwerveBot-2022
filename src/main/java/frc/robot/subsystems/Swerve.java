package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.limeLight;
public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  // public Pigeon2 gyro;
  public AHRS gyro;
  public Field2d m_field = new Field2d();
  // limeLight curr_lime;
  public Swerve() {
    // gyro = new Pigeon2(Constants.Swerve.pigeonID);
    // gyro.configFactoryDefault();
    // zeroGyro();
    gyro = new AHRS(SPI.Port.kMXP);
    //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
    
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod1.constants),
          new SwerveModule(1, Constants.Swerve.Mod0.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
      // curr_lime=new limeLight();
        
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            true
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
    resetOdometry(new Pose2d(0, 0,new Rotation2d(0)));
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.zeroYaw();;
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getStates());
    // System.out.println(curr_lime.getTransforemd());
    for (SwerveModule mod : mSwerveMods) {
      // SmartDashboard.putNumber(
          // "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      // SmartDashboard.putNumber(
      //     "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      //     m_field.setRobotPose(swerveOdometry.getPoseMeters());
      //     SmartDashboard.putData("Field", m_field);

    }
  }
}
