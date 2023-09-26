// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera m_camera;
  PhotonPipelineResult m_result;
  PhotonTrackedTarget m_bestTarget;
  /** Creates a new PhotonVision. */
  public CameraSubsystem(String CameraName) {
    m_camera = new PhotonCamera(CameraName);
  }

  public boolean hasTargets() {
    return m_result.hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    return m_result.getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return m_result.getBestTarget();
  }

  public void takeSnapshot() {
    m_camera.takeInputSnapshot();
  }

  public double getPitch() {
    return m_bestTarget.getPitch();
  }

  public double getYaw() {
    return m_bestTarget.getYaw();
  }

  public double getSkew() {
    return m_bestTarget.getSkew();
  }

  public double getArea() {
    return m_bestTarget.getArea();
  }

  public List<TargetCorner> getCorners() {
    return m_bestTarget.getDetectedCorners();
  }

  public Transform3d getDistance() {
    return m_bestTarget.getBestCameraToTarget();
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult(); 
    if (hasTargets()) {
      m_bestTarget = m_result.getBestTarget();
    }
  }
}
