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
import frc.robot.subsystems.Camera1Subsystem;

public class CameraTest extends Command {
  private final Camera1Subsystem m_camera;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private Timer m_timer;
  public CameraTest(Camera1Subsystem camera) {
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
    m_camera = camera;
    addRequirements(camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> robotPoseEstimator = m_camera.getFieldRelativePoseEstimator();
    if (robotPoseEstimator.isEmpty()) return;
    SmartDashboard.putString("Estimator Pose", robotPoseEstimator.get().estimatedPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPoseEstimator.get().estimatedPose.getRotation().getAngle()*Math.PI));
    m_poseEstimator.update(new Rotation2d(), 0, 0);
    m_poseEstimator.addVisionMeasurement(robotPoseEstimator.get().estimatedPose.toPose2d(), robotPoseEstimator.get().timestampSeconds);
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
