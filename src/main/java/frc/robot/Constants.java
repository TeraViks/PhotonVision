// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  public static class PhotonVisionConstants {
    public static final double kCameraHeightMeters1 = 0.0;
    public static final double kCameraHeightMeters2 = 0.0;
    public static final double kCameraHeightMeters3 = 0.0;
    public static final double KCameraPitchRadians = 0.0;
    public static final double kTargetHeightMeters1 = 0.0;
    public static final double kTargetHeightMeters2 = 0.0;
    public static final double kTargetHeightMeters3 = 0.0;
    public static final double kTargetHeightMeters4 = 0.0;
    public static final double kTargetHeightMeters5 = 0.0;
    public static final double kTargetHeightMeters6 = 0.0;
    public static final double kTargetHeightMeters7 = 0.0;
    public static final double kTargetHeightMeters8 = 0.0;
    public static double findCameraHeight(String cameraName) {
      if (cameraName == "1") {
        return kCameraHeightMeters1;
      }
      else if (cameraName == "2") {
        return kCameraHeightMeters2;
      }
      else if (cameraName == "3") {
        return kCameraHeightMeters3;
      }
      else {
        throw new Error("Invalid Camera ID");
      }
    }
    public static double findTargetHeight(int targetID) {
      if (targetID == 1) {
        return kTargetHeightMeters1;
      }
      else if (targetID == 2) {
        return kTargetHeightMeters2;
      }
      else if (targetID == 3) {
        return kTargetHeightMeters3;
      }
      else if (targetID == 4) {
        return kTargetHeightMeters4;
      }
      else if (targetID == 5) {
        return kTargetHeightMeters5;
      }
      else if (targetID == 6) {
        return kTargetHeightMeters6;
      }
      else if (targetID == 7) {
        return kTargetHeightMeters7;
      }
      else if (targetID == 8) {
        return kTargetHeightMeters8;
      }
      else {
        throw new Error("Invalid Target ID");
      }
    }
  }
  }
}
