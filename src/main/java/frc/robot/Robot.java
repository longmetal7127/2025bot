// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoLoop;

import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser.AutoRoutineGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.configs.Constants;
import frc.robot.configs.Swerve;
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
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private DriveTrain driveTrain = new DriveTrain();
  private AutoFactory autoFactory = Choreo.createAutoFactory(
      driveTrain,
      driveTrain::getPose,
      new AutoController(driveTrain),
      () -> {
        return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Red);
      },
      new AutoBindings());
  private AutoChooser autoChooser;
  public static CommandJoystick joystick = new CommandJoystick(
      Constants.OperatorConstants.kDriverJoystickPort);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureBindings();
    autoChooser = new AutoChooser(autoFactory, "");
    autoChooser.addAutoRoutine("auto", this::auto);
    driveTrain.setDefaultCommand(
        new RunCommand(
            () -> {
              double multiplier = (((joystick.getThrottle() * -1) + 1) / 2); // turbo mode
              double z = joystick.getZ() * -.9;

              driveTrain.drive(
                  MathUtil.applyDeadband(
                      joystick.getY() * -multiplier,
                      OperatorConstants.kDriveDeadband),
                  MathUtil.applyDeadband(
                      joystick.getX() * -multiplier,
                      OperatorConstants.kDriveDeadband),
                  MathUtil.applyDeadband(z, OperatorConstants.kDriveDeadband),
                  true);
            },
            driveTrain));

  }

  public void configureBindings() {

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
  @Override
  public void simulationPeriodic() {
  }

  public Command getAutonomousCommand() {
    return Commands.run(() -> {

    });
  }

  public Command auto(final AutoFactory factory) {
    final AutoLoop routine = factory.newLoop("auto");

    final AutoTrajectory trajectory = factory.trajectory("auto", routine);

    // entry point for the auto
    // resets the odometry to the starting position,
    // then shoots the starting note,
    // then runs the trajectory to the first close note while extending the intake
    routine.enabled()
        .onTrue(
            driveTrain.cmdResetOdometry(
                trajectory.getInitialPose()
                    .orElseGet(
                        () -> {
                          routine.kill();
                          return new Pose2d();
                        }))
                .withName("auto entry point"));

    return routine.cmd().withName("auto");
  }

}
