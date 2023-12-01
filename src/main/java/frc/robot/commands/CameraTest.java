// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera1Subsystem;

public class CameraTest extends Command {
  private final Camera1Subsystem m_camera;
  private Timer m_timer;
  public CameraTest(Camera1Subsystem camera) {
    m_timer = new Timer();
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
    PhotonTrackedTarget target;
      Optional<PhotonTrackedTarget> Optionaltarget = m_camera.getLowestAmbiguityTarget();
      if (Optionaltarget.isEmpty()) return;
      target = Optionaltarget.get();

      Pose3d robotPose = m_camera.getFieldRelativePose(target);
      Optional<EstimatedRobotPose> robotPoseEstimator = m_camera.getFieldRelativePoseEstimator();
      if (robotPoseEstimator.isEmpty()) return;
      SmartDashboard.putString("Estimator Pose", robotPoseEstimator.get().estimatedPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPoseEstimator.get().estimatedPose.getRotation().getAngle()*Math.PI));
      SmartDashboard.putString("No Estimator Pose", robotPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPose.getRotation().getAngle()));
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
