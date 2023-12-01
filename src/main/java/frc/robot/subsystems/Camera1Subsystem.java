// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonVisionConstants;

public class Camera1Subsystem extends SubsystemBase {
  // private final double kAprilTagOffsetMeters = 0.0762;
  private final double kAprilTagOffsetMeters = 0.0;

  PhotonCamera m_camera;
  PhotonPoseEstimator m_photonPoseEstimator;
  PhotonPipelineResult m_result;
  Optional<PhotonTrackedTarget> m_lowestAmbiguityTarget;
  AprilTag aprilTag1 = new AprilTag(1, new Pose3d(new Translation3d(4.0, 0.5+kAprilTagOffsetMeters, PhotonVisionConstants.kTarget1HeightMeters), new Rotation3d(0.0, 0.0, Math.PI)));
  AprilTag aprilTag2 = new AprilTag(2, new Pose3d(new Translation3d(4.0, 2.5+kAprilTagOffsetMeters, PhotonVisionConstants.kTarget2HeightMeters), new Rotation3d(0.0, 0.0, Math.PI)));
  AprilTag aprilTag3 = new AprilTag(3, new Pose3d(new Translation3d(4.0, 1.5+kAprilTagOffsetMeters, PhotonVisionConstants.kTarget3HeightMeters), new Rotation3d(0.0, 0.0, Math.PI)));
  List<AprilTag> aprilTagList = Arrays.asList(aprilTag1, aprilTag2, aprilTag3);

  AprilTagFieldLayout m_aprilTagFieldLayout = 
    new AprilTagFieldLayout(
      aprilTagList, FieldConstants.kFieldLength, FieldConstants.kFieldWidth
    );

  //Constructor
  public Camera1Subsystem(String CameraName) {
    m_camera = new PhotonCamera(CameraName);
    m_photonPoseEstimator = 
    new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_camera,
      PhotonVisionConstants.kCamera1ToRobotOffset
    );
  }

  public PoseStrategy getEstimatorStrategy() {
    return m_photonPoseEstimator.getPrimaryStrategy();
  }

  public void setLED(VisionLEDMode mode) {
    m_camera.setLED(mode);
  }

  public boolean hasTargets() {
    return m_result.hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    return m_result.getTargets();
  }

  public void takeSnapshot() {
    m_camera.takeInputSnapshot();
  }

  public Optional<EstimatedRobotPose> getFieldRelativePoseEstimator() {
    return m_photonPoseEstimator.update(m_result);
  }

  public double getDistanceToTarget(PhotonTrackedTarget target) {
    return (
      PhotonUtils.calculateDistanceToTargetMeters(
        PhotonVisionConstants.kCamera1HeightMeters,
        PhotonVisionConstants.findTargetHeight(target.getFiducialId()),
        PhotonVisionConstants.KCameraPitchRadians,
        Units.degreesToRadians(target.getPitch())
      )
    );
  }

  public Pose3d getFieldRelativePose(PhotonTrackedTarget target) {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        m_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
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
