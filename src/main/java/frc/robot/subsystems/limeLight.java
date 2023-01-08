// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limeLight extends SubsystemBase {
  PhotonCamera camera;
  Transform3d relative;

  public Transform3d getTransforemd()
  {
       return relative;
  }
  /** Creates a new limeLight. */
  public limeLight() 
  {
    camera = new PhotonCamera("ninjas4744");
  }

  @Override
  public void periodic() {
    PhotonPipelineResult res = camera.getLatestResult();
    SmartDashboard.putBoolean("target", res.hasTargets());
    if (res.hasTargets()){
    // relative =  camera.getLatestResult().targets.get(0).getArea();
      // System.out.println(res.getBestTarget());
      SmartDashboard.putNumber("cornerX", res.getBestTarget().getCorners().get(0).x);
      SmartDashboard.putNumber("cornerY", res.getBestTarget().getCorners().get(0).x);
      SmartDashboard.putNumber("Skew", res.getBestTarget().getSkew());
      System.out.println(res.getBestTarget().getSkew());// SmartDashboard.putNumber("X", res.getBestTarget().getBestCameraToTarget().getX());
      // SmartDashboard.putNumber("Y", res.getBestTarget().getBestCameraToTarget().getY());
      // SmartDashboard.putNumber("Z", res.getBestTarget().getBestCameraToTarget().getZ());
    }
    // This method will be called once per scheduler run
  }
}
