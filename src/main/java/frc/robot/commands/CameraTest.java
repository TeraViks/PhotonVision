// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;

public class CameraTest extends Command {
  private final CameraSubsystem m_photonCameras;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private Timer m_timer;
  public CameraTest(CameraSubsystem cameras) {
    m_timer = new Timer();
    m_poseEstimator =
      new DifferentialDrivePoseEstimator(
          new DifferentialDriveKinematics(0),
          new Rotation2d(),
          0,
          0,
          new Pose2d(),
          VecBuilder.fill(0.005, 0.005, Math.toRadians(1)),
          VecBuilder.fill(0.05, 0.05, Math.toRadians(5)));
    m_photonCameras = cameras;
    addRequirements(cameras);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> robotPoseEstimatorCam2 = m_photonCameras.getFieldRelativePoseEstimatorCam2();
    if (!robotPoseEstimatorCam2.isEmpty()) {
      m_poseEstimator.addVisionMeasurement(robotPoseEstimatorCam2.get().estimatedPose.toPose2d(), robotPoseEstimatorCam2.get().timestampSeconds);
      SmartDashboard.putString("Estimator Pose 2", robotPoseEstimatorCam2.get().estimatedPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPoseEstimatorCam2.get().estimatedPose.getRotation().getAngle()*Math.PI));
    }

    Optional<EstimatedRobotPose> robotPoseEstimatorCam1 = m_photonCameras.getFieldRelativePoseEstimatorCam1();
    if (!robotPoseEstimatorCam1.isEmpty()) {
      m_poseEstimator.addVisionMeasurement(robotPoseEstimatorCam1.get().estimatedPose.toPose2d(), robotPoseEstimatorCam1.get().timestampSeconds);
      SmartDashboard.putString("Estimator Pose 1", robotPoseEstimatorCam1.get().estimatedPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPoseEstimatorCam1.get().estimatedPose.getRotation().getAngle()*Math.PI));
    }

    m_poseEstimator.update(new Rotation2d(), 0, 0);

    SmartDashboard.putString("DifferentialDriveEstimatorPose", m_poseEstimator.getEstimatedPosition().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
