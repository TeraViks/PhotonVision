// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonVisionConstants;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera m_camera;
  PhotonPipelineResult m_result;
  PhotonTrackedTarget m_bestTarget;
  AprilTagFieldLayout m_aprilTagFieldLayout = 
    new AprilTagFieldLayout(
      null, FieldConstants.kFieldLength, FieldConstants.kFieldWidth
    );

  public CameraSubsystem(String CameraName) {
    m_camera = new PhotonCamera(CameraName);
  }

  public boolean hasTargets() {
    return m_result.hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    return m_result.getTargets();
  }

  public int getTagID() {
    return m_bestTarget.getFiducialId();
  }

  public int getTagID(PhotonTrackedTarget target) {
    return target.getFiducialId();
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

  public double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getYaw() {
    return m_bestTarget.getYaw();
  }

  public double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getSkew() {
    return m_bestTarget.getSkew();
  }

  public double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  public double getArea() {
    return m_bestTarget.getArea();
  }

  public double GetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public List<TargetCorner> getCorners() {
    return m_bestTarget.getDetectedCorners();
  }

  public List<TargetCorner> getCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  public Transform3d getBestCameraToTarget() {
    return m_bestTarget.getBestCameraToTarget();
  }

  public Transform3d getBestCameraToTarget(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  public double getDistanceToTarget() {
    return (
      PhotonUtils.calculateDistanceToTargetMeters(
        PhotonVisionConstants.findCameraHeight(m_camera.getName()),
        PhotonVisionConstants.findTargetHeight(m_bestTarget.getFiducialId()),
        PhotonVisionConstants.KCameraPitchRadians,
        Units.degreesToRadians(getPitch())
      )
    );
  }

  public double getDistanceToTarget(PhotonTrackedTarget target) {
    return (
      PhotonUtils.calculateDistanceToTargetMeters(
        PhotonVisionConstants.findCameraHeight(m_camera.getName()),
        PhotonVisionConstants.findTargetHeight(target.getFiducialId()),
        PhotonVisionConstants.KCameraPitchRadians,
        Units.degreesToRadians(getPitch(target))
      )
    );
  }

  public Pose3d getFieldRelativePose() {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        getBestCameraToTarget(),
        m_aprilTagFieldLayout.getTagPose(getTagID()).get(),
        PhotonVisionConstants.kCameraToRobotOffset
      );
  }

  public Pose3d getFieldRelativePose(PhotonTrackedTarget target) {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        getBestCameraToTarget(target),
        m_aprilTagFieldLayout.getTagPose(getTagID(target)).get(),
        PhotonVisionConstants.kCameraToRobotOffset
      );
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult(); 
    if (hasTargets()) {
      m_bestTarget = m_result.getBestTarget();
    }
  }
}
