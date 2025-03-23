// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.FiducialPoseEstimator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final boolean kLogitech = false;
    public static final int kDriverJoystickPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kLogitechDeadband = 0.1;

  }

  public static final double FIELD_LENGTH_METERS = FiducialPoseEstimator.tagLayout.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FiducialPoseEstimator.tagLayout.getFieldWidth();
  public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);

  public static final double FRAME_WIDTH_METERS = Units.inchesToMeters(29);
  public static final double FRAME_LENGTH_METERS = Units.inchesToMeters(29);
  public static final double BUMPER_THICKNESS_METERS = Units.inchesToMeters(4.25);

  public static double tag18X = FiducialPoseEstimator.tagLayout.getTagPose(18).get().getX();
  public static double tag21X = FiducialPoseEstimator.tagLayout.getTagPose(21).get().getX();
  public static Translation2d BLUE_REEF = new Translation2d((tag18X + tag21X) / 2, Constants.FIELD_WIDTH_METERS / 2);
  public static Translation2d RED_REEF = BLUE_REEF.rotateAround(FIELD_CENTER, Rotation2d.k180deg);


}
