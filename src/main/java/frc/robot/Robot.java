// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.urcl.URCL;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.configs.Constants;
import frc.robot.configs.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private DriveTrain driveTrain = new DriveTrain();
  private AutoFactory autoFactory;
  private AutoChooser autoChooser;
  public static CommandJoystick joystick = new CommandJoystick(
      Constants.OperatorConstants.kDriverJoystickPort);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    DataLogManager.start();
    URCL.start();
    autoFactory = new AutoFactory(
        driveTrain::getPose,
        driveTrain::resetOdometry,
        new AutoController(driveTrain),
        true, driveTrain);
    configureBindings();
    autoChooser = new AutoChooser();
    autoChooser.addRoutine("auto", this::auto);

    driveTrain.setDefaultCommand(
        new RunCommand(
            () -> {

              double multiplier = (((joystick.getThrottle() * -1) + 1) / 2); // turbo mode
              double z = joystick.getZ();
              double x = joystick.getX();
              double y = joystick.getY();

              // limiting x/y on input methods
              x = Math.sin(Math.atan2(x, y)) * Math.min(Math.max(Math.abs(y), Math.abs(x)), 1);
              y = Math.cos(Math.atan2(x, y)) * Math.min(Math.max(Math.abs(y), Math.abs(x)), 1);
              double deadband = OperatorConstants.kLogitech ? OperatorConstants.kLogitechDeadband
                  : OperatorConstants.kDriveDeadband;

              driveTrain.drive(
                  MathUtil.applyDeadband(
                      y * -multiplier,
                      deadband),
                  MathUtil.applyDeadband(
                      x * -multiplier,
                      deadband),
                  MathUtil.applyDeadband(z * -1, deadband),
                  true);
            },
            driveTrain));
    Epilogue.bind(this);

  }

  public void configureBindings() {
    joystick.trigger().onTrue(Commands.run(() -> {
      driveTrain.zeroHeading();
    }));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  // simulation period method in your Robot.java
  @Override
  public void simulationPeriodic() {
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {

    });
  }

  public AutoRoutine auto() {
    final AutoRoutine routine = autoFactory.newRoutine("auto");

    final AutoTrajectory trajectory = routine.trajectory("auto");

    // entry point for the auto
    // resets the odometry to the starting position,
    // then shoots the starting note,
    // then runs the trajectory to the first close note while extending the intake
    routine.active()
        .onTrue(
            driveTrain.cmdResetOdometry(
                trajectory.getInitialPose()
                    .orElseGet(
                        () -> {
                          routine.kill();
                          return new Pose2d();
                        }))
                .withName("auto entry point"));

    return routine;
  }

}
