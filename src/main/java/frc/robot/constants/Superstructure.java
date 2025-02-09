// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class Superstructure {

  public static final class CANIds {

    public static final int kElevatorMotorCanId = 9;
    public static final int kElevatorMotorFollowerCanId = 10;

    public static final int kArmMotorCanId = 11;
  }

  public static final class Configs {

    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorFollowerConfig =
      new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
      armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12);

      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60); // .voltageCompensation(12);
      elevatorFollowerConfig.follow(CANIds.kElevatorMotorCanId);
    }
  }

  public static final class ElevatorConstants {

    public static final double kElevatorkG = 0.762;
    public static final double kElevatorkS = 0;
    public static final double kElevatorkV = 0.762;
    public static final double kElevatorkA = 0.11;

    public static final double kElevatorkP = 5;
    public static final double kElevatorkI = 0;
    public static final double kElevatorkD = 0.23;

    public static final double kElevatorMaxVelocity = Meters.of(4)
      .per(Second)
      .in(MetersPerSecond);
    public static final double kElevatorMaxAcceleration = Meters.of(8)
      .per(Second)
      .per(Second)
      .in(MetersPerSecondPerSecond);
  }

  public static final class ArmConstants {

    public static final double kArmkG = 0.762;
    public static final double kArmkS = 0;
    public static final double kArmkV = 0.762;
    public static final double kArmkA = 0.11;

    public static final double kArmkP = 2.0691;
    public static final double kArmkI = 0;
    public static final double kArmkD = 0.0;

    public static final double kArmMaxVelocityRPM = 15;
    public static final double kArmMaxAccelerationRPMperSecond = 30;
  }

  public static final class PhysicalRobotConstants {

    public static final double kElevatorGearing = 3.75; // 5:1 + 24:18
    public static final double kCarriageMass = 6.80388555;
    public static final double kElevatorDrumRadius =
      Units.inchesToMeters(2.5) / 2.0; // m
    public static final double kMinElevatorCarriageHeightMeters = 0.2286; // m
    public static final double kMinElevatorStage1HeightMeters = 0.9652; // m
    public static final double kMaxElevatorCarriageHeightMeters = 0.9144; // m
    public static final double kMaxElevatorStage1HeightMeters = 1.72794689; // m
    public static final double kCarriageTravelHeightMeters =
      kMaxElevatorCarriageHeightMeters - kMinElevatorCarriageHeightMeters;
    public static final double kStage1TravelHeightMeters =
      kMaxElevatorStage1HeightMeters - kMinElevatorStage1HeightMeters;

    public static final Distance kArmLength = Meters.of(0.45076397);
    public static final Mass kArmMass = Pounds.of(7.5258052);
    public static final double kArmReduction = 32; // TODO: double check
    public static final double kMinAngleRads = -8 * Math.PI;
    public static final double kMaxAngleRads = 8 * Math.PI;
  }

  public static final class Setpoints {

    public static final double kZero = 0;
  }

  public static final class Mechanisms {

    public static final Mechanism2d m_mech2d = new Mechanism2d(10, 10);
    public static final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot(
      "ElevatorArm Root",
      25,
      0
    );
    public static final MechanismLigament2d m_elevatorStage1Mech2d =
      m_mech2dRoot.append(
        new MechanismLigament2d(
          "Elevator Stage 1",
          PhysicalRobotConstants.kMinElevatorStage1HeightMeters,
          90
        )
      );
    public static final MechanismLigament2d m_elevatorCarriageMech2d =
      m_elevatorStage1Mech2d.append(
        new MechanismLigament2d(
          "Elevator 6Carriage",
          PhysicalRobotConstants.kMinElevatorCarriageHeightMeters,
          0
        )
      );

    public static final MechanismLigament2d m_armMech2d =
      m_elevatorCarriageMech2d.append(
        new MechanismLigament2d(
          "Arm",
          PhysicalRobotConstants.kArmLength.in(Meters),
          180 -
          Units.radiansToDegrees(PhysicalRobotConstants.kMinAngleRads) -
          180
        )
      );
  }
}
