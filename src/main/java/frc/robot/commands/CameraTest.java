// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera1Subsystem;

public class CameraTest extends CommandBase {
  private final Camera1Subsystem m_camera;
  private Timer m_timer;
  private class Value {
    public final double value;
    public Value(double v) {
      value = v;
    }
  }
  // private double m_xValueList[];
  // private double m_yValueList[];
  private List<Value> m_xValueList = new ArrayList<Value>();
  private List<Value> m_yValueList = new ArrayList<Value>();
  public CameraTest(Camera1Subsystem camera) {
    m_timer = new Timer();
    m_camera = camera;
    addRequirements(camera);
  }

  private static double calculateStandardDeviation(List<Value> list) {
    // get the sum of array
    int length = 0;
    double sum = 0.0;
    for (Value v : list) {
        sum += v.value;
        length += 1;
    }

    if (length == 0) {
      return 0.0;
    }

    // get the mean of array
    double mean = sum / length;

    // calculate the standard deviation
    double standardDeviation = 0.0;
    for (Value num : list) {
        standardDeviation += Math.pow(num.value - mean, 2);
    }

    return Math.sqrt(standardDeviation / length);
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
    // if (m_timer.advanceIfElapsed(1)) {
      Optional<PhotonTrackedTarget> Optionaltarget = m_camera.getLowestAmbiguityTarget();
      if (Optionaltarget.isEmpty()) return;
      target = Optionaltarget.get();

      Pose3d robotPose = m_camera.getFieldRelativePose(target);
      Optional<EstimatedRobotPose> robotPoseEstimator = m_camera.getFieldRelativePoseEstimator();
      if (robotPoseEstimator.isEmpty()) return;
      // m_xValueList.add(new Value(robotPoseEstimator.get().estimatedPose.getX()));
      // m_yValueList.add(new Value(robotPoseEstimator.get().estimatedPose.getY()));
      SmartDashboard.putString("Estimator Pose", robotPoseEstimator.get().estimatedPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPoseEstimator.get().estimatedPose.getRotation().getAngle()*Math.PI));
      SmartDashboard.putString("No Estimator Pose", robotPose.getTranslation().toString() + " | Rotation: " + Math.toDegrees(robotPose.getRotation().getAngle()));
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(m_camera.getEstimatorStrategy().toString());
    System.out.println("X: " + calculateStandardDeviation(m_xValueList));
    System.out.println("Y: " + calculateStandardDeviation(m_yValueList));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_timer.hasElapsed(10)) return true;
    // return false;
    return false;
  }
}
