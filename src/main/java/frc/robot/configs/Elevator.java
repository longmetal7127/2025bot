// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public class Elevator {
    public static final class CANIds {
        public static final int kElevatorMotorCanId = 9;
        public static final int kElevatorMotorFollowerCanId = 10;

        public static final int kArmMotorCanId = 11;
    }

    public static final class Configs {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

        static {
            // Configure basic settings of the arm motor
            armConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

            /*
             * Configure the closed loop controller. We want to make sure we set the
             * feedback sensor as the primary encoder.
             */
            armConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(0.1)
                    .outputRange(-1, 1).maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(2000)
                    .maxAcceleration(10000)
                    .allowedClosedLoopError(0.25);

            elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

           
            elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    .p(0.07)
                    .d(0.23)
                    .outputRange(-1, 1)
                    .maxMotion // Set MAXMotion parameters for position control
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.5);
            elevatorFollowerConfig.follow(CANIds.kElevatorMotorCanId);

        }
    }

    public static final class SimulationRobotConstants {
        public static final double kPixelsPerMeter = 1;

        public static final double kElevatorGearing = 5; // 5:1
        public static final double kCarriageMass = 8.51313073; 
        public static final double kElevatorDrumRadius = 0.063500 / 2.0; // m
        public static final double kMinElevatorCarriageHeightMeters = 0.2286; // m
        public static final double kMinElevatorStage1HeightMeters = 0.9652; // m
        public static final double kMaxElevatorCarriageHeightMeters = 0.9144; // m
        public static final double kMaxElevatorStage1HeightMeters = 1.72794689; // m
        public static final double kCarriageTravelHeightMeters = kMaxElevatorCarriageHeightMeters
                - kMinElevatorCarriageHeightMeters;
        public static final double kStage1TravelHeightMeters = kMaxElevatorStage1HeightMeters
                - kMinElevatorStage1HeightMeters;


        public static final Distance kArmLength = Meters.of(0.45076397);
        public static final Mass kArmMass = Pounds.of(7.5258052);
        public static final double kArmReduction = 32; // TODO: double check
        public static final double kMinAngleRads = 0;
        public static final double kMaxAngleRads = 0;

    }

    public static final class Setpoints {
        public static final double kZero = 0;
    }

}
