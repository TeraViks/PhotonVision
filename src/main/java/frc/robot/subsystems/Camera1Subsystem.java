// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

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

public class Camera1Subsystem extends SubsystemBase {
  PhotonCamera m_camera;
  PhotonPipelineResult m_result;
  Optional<PhotonTrackedTarget> m_lowestAmbiguityTarget;
  //TODO: Fill in the null for the apriltag
  AprilTagFieldLayout m_aprilTagFieldLayout = 
    new AprilTagFieldLayout(
      null, FieldConstants.kFieldLength, FieldConstants.kFieldWidth
    );

  public Camera1Subsystem(String CameraName) {
    m_camera = new PhotonCamera(CameraName);
  }

  public boolean hasTargets() {
    return m_result.hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    return m_result.getTargets();
  }

  public int getTagID(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  public void takeSnapshot() {
    m_camera.takeInputSnapshot();
  }

  public double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  public double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public List<TargetCorner> getCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  public Transform3d getBestCameraToTarget(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
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

  public Pose3d getFieldRelativePose(PhotonTrackedTarget target) {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        getBestCameraToTarget(target),
        m_aprilTagFieldLayout.getTagPose(getTagID(target)).get(),
        PhotonVisionConstants.kCamera1ToRobotOffset
      );
  }

  private Optional<PhotonTrackedTarget> getLowestAmbiguityTargetImpl(List<PhotonTrackedTarget> targetList) {
    Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
    for (int i=0; i<targetList.size(); i++) {
      PhotonTrackedTarget currentTarget = targetList.get(i);
      if (currentTarget.getPoseAmbiguity() == -1 || currentTarget.getPoseAmbiguity() >= 0.2) continue;
      if (bestTarget.isEmpty()) {
        bestTarget = Optional.of(currentTarget);
        continue;
      }
      if (currentTarget.getPoseAmbiguity() < bestTarget.get().getPoseAmbiguity()) {
        bestTarget = Optional.of(currentTarget);
      }
    }
    return bestTarget;
  }

  public Optional<PhotonTrackedTarget> getLowestAmbiguityTarget() {
    return m_lowestAmbiguityTarget;
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult(); 
    m_lowestAmbiguityTarget = getLowestAmbiguityTargetImpl(getAllTargets());
  }
}
