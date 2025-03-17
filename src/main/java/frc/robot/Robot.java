// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.Constants;
import frc.robot.constants.Swerve.DriveSetpoints;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.superstructure.Wrist;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.Take;
import frc.robot.subsystems.superstructure.Wrist.WristState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.util.Tracer;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

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
  public Elevator elevator = new Elevator();
  public Wrist Wrist = new Wrist();
  private Take take = new Take();
  private LED led = new LED();

  private AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  public static CommandJoystick joystick = new CommandJoystick(
      Constants.OperatorConstants.kDriverJoystickPort);
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  PowerDistribution pdh = new PowerDistribution(20, ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    DataLogManager.start();
    URCL.start();
    SignalLogger.start();
    // DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    // DogLog.setPdh(pdh);

    autoFactory = new AutoFactory(
        driveTrain::getPose,
        driveTrain::resetOdometry,
        new AutoController(driveTrain),
        true,
        driveTrain);
    // configureBindingsSysid();
    configureBindings();
    autoChooser = new AutoChooser();

    autoChooser.addCmd("One piece A", this.onePiece(DriveSetpoints.A));
    autoChooser.addCmd("One piece B", this.onePiece(DriveSetpoints.B));
    autoChooser.addCmd("One piece C", this.onePiece(DriveSetpoints.C));
    autoChooser.addCmd("One piece D", this.onePiece(DriveSetpoints.D));
    autoChooser.addCmd("One piece E", this.onePiece(DriveSetpoints.E));
    autoChooser.addCmd("One piece F", this.onePiece(DriveSetpoints.F));
    autoChooser.addCmd("One piece G", this.onePiece(DriveSetpoints.G));
    autoChooser.addCmd("One piece H", this.onePiece(DriveSetpoints.H));
    autoChooser.addCmd("One piece I", this.onePiece(DriveSetpoints.I));
    autoChooser.addCmd("One piece J", this.onePiece(DriveSetpoints.J));
    autoChooser.addCmd("One piece K", this.onePiece(DriveSetpoints.K));
    autoChooser.addCmd("One piece L", this.onePiece(DriveSetpoints.L));

    autoChooser.addCmd("Three piece Left", this::threePieceLeft);
    autoChooser.addCmd("Three piece Right", this::threePieceRight);
    autoChooser.addCmd("test", this::test);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    driveTrain.setDefaultCommand(driveTrain.joystickDrive(joystick::getX, joystick::getY, joystick::getZ,
        Optional.of(elevator::getLinearPositionMeters)));

    RobotModeTriggers.disabled().onTrue(led.setRainbow().ignoringDisable(true));
    take.hasCoral.and(RobotModeTriggers.disabled().negate()).onTrue(led.setGreen());
    take.hasCoral.negate().and(RobotModeTriggers.disabled().negate()).onTrue(led.setDefault());

    Epilogue.bind(this);
  }

  public void configureBindingsSysid() {
    // joystick.trigger().onTrue(Wrist.wristToPosition(WristState.Safe).andThen(elevator.runSysIdRoutine()));
    joystick.trigger().onTrue(Wrist.runSysIdRoutine());

  }

  public void configureBindings() {
    joystick.button(5).onTrue(driveTrain.zeroHeading());

    joystick
        .button(2)
        .onTrue(
            Commands.sequence(
                Wrist.wristToPosition(WristState.Safe),

                elevator.elevatorToPosition(ElevatorState.Level2),
                Wrist.wristToPosition(WristState.LevelNormal)));
    joystick
        .button(3)
        .onTrue(
            Commands.sequence(
                Wrist.wristToPosition(WristState.Safe),
                elevator.elevatorToPosition(ElevatorState.Level3),
                Wrist.wristToPosition(WristState.LevelNormal)));

    joystick
        .button(4)
        .onTrue(
            Commands.sequence(
                Wrist.wristToPosition(WristState.Safe),
                elevator.elevatorToPosition(ElevatorState.Level4),
                Wrist.wristToPosition(WristState.Level4)));

    joystick.povDown().or(joystick.povDownLeft()).or(joystick.povDownRight()).onTrue(intake());
    joystick.trigger().whileTrue(take.runTakeMotor());
    joystick.button(11).onTrue(
        driveTrain.autoAlignChooseSetpoint(true,
            Optional.of(joystick::getX),
            Optional.of(joystick::getX),
            Optional.of(joystick::getZ)).withDeadline(autoAlignEnd()));
    joystick.button(12).onTrue(
        driveTrain.autoAlignChooseSetpoint(false,
            Optional.of(joystick::getX),
            Optional.of(joystick::getX),
            Optional.of(joystick::getZ)).withDeadline(autoAlignEnd()));

    // joystick.povRight().onTrue(Wrist.runSysIdRoutine());
    // elevator.atZeroNeedReset.onTrue(elevator.zero());
    joystick.button(13).onTrue(elevator.setSetpointCommand(ElevatorState.Zeroing).until(elevator.basicallyNotMoving)
        .andThen(elevator.zero()).withTimeout(14).andThen(elevator.elevatorToPosition(ElevatorState.Handoff)));
  }

  public Command autoAlignEnd() {
    return Commands.race(
        Commands.waitUntil(driveTrain.atSetpoint),
        Commands.sequence(
            Commands.waitUntil(elevator.atSetpoint(ElevatorState.Handoff).negate()),
            Commands.waitUntil(elevator.atSetpoint(ElevatorState.Handoff))),
        Commands.waitUntil(joystick.button(10)));
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
    Tracer.traceFunc("CommandScheduler", scheduler::run);

  }

  public Supplier<Command> onePiece(DriveSetpoints setpoint) {
    return () -> {
      return driveTrain
          .autoAlign(() -> setpoint, Optional.empty(), Optional.empty(), Optional.empty())
          .until(driveTrain.atSetpointAuto)
          .andThen(Wrist.wristToPosition(WristState.Safe))
          .andThen(elevator.elevatorToPosition(ElevatorState.Level4))
          .andThen(Wrist.wristToPosition(WristState.Level4))
          .andThen(shootNote());
    };
  }

  public Command test() {
    return sourceIntake(false);
  }

  public Command threePieceLeft() {
    return reefCycle(DriveSetpoints.I, ElevatorState.Level4, WristState.Level4)
        .andThen(sourceIntake(true))
        .andThen(reefCycle(DriveSetpoints.J, ElevatorState.Level4, WristState.Level4))
        .andThen(sourceIntake(true))
        .andThen(reefCycle(DriveSetpoints.A, ElevatorState.Level4, WristState.Level4))
        .andThen(sourceIntake(true));
  }

  public Command threePieceRight() {
    return reefCycle(DriveSetpoints.E, ElevatorState.Level4, WristState.Level4)
        .andThen(sourceIntake(false))
        .andThen(reefCycle(DriveSetpoints.F, ElevatorState.Level4, WristState.Level4))
        .andThen(sourceIntake(false))
        .andThen(reefCycle(DriveSetpoints.B, ElevatorState.Level4, WristState.Level4))
        .andThen(sourceIntake(false));
  }

  public Command shootNote() {
    return take.runTakeMotor().withDeadline(
        Commands.sequence(
            Commands.waitUntil(take.hasCoral),
            Commands.waitUntil(take.hasCoral.negate()),
            Commands.waitSeconds(0.2)));
  };

  public Command reefCycle(DriveSetpoints driveSetpoint, ElevatorState elevatorSetpoint, WristState wristState) {
    return driveTrain.autoAlign(
        () -> driveSetpoint,
        Optional.empty(),
        Optional.empty(),
        Optional.empty()).until(driveTrain.atSetpointAuto)
        .andThen(Wrist.wristToPosition(WristState.Safe))
        .andThen(elevator.elevatorToPosition(elevatorSetpoint))
        .andThen(shootNote())
        .andThen(Wrist.wristToPosition(WristState.Safe)).andThen(
            elevator.elevatorToPosition(ElevatorState.Handoff))
        .andThen(Wrist.setSetpointCommand(WristState.Handoff)); // make sure to doublecheck it could be sketch but i
                                                                // think it should be fineee
  }

  public Command sourceIntake(boolean left) {
    return driveTrain.autoAlign(() -> left ? DriveSetpoints.LEFT_HP : DriveSetpoints.RIGHT_HP, Optional.empty(),
        Optional.empty(), Optional.empty()).until(driveTrain.atSetpointSource)
        .andThen(intake());
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
    System.out.print("\n\nAUTO CALLED\n\n");
    final AutoRoutine routine = autoFactory.newRoutine("testauto");

    final AutoTrajectory trajectory = routine.trajectory("path3");

    // entry point for the auto
    // resets the odometry to the starting position,
    // then shoots the starting note,
    // then runs the trajectory to the first close note while extending the intake
    routine
        .active()
        .onTrue(
            driveTrain
                .cmdResetOdometry(
                    trajectory
                        .getInitialPose()
                        .orElseGet(() -> {
                          routine.kill();
                          return new Pose2d();
                        }))
                .withName("auto entry point"));

    return routine;
  }

  public Command intake() {
    return Commands.sequence(
        Wrist.wristToPosition(WristState.Safe)
            .unless(Wrist.atSetpoint(WristState.Handoff).and(elevator.atSetpoint(ElevatorState.Handoff))),
        elevator.elevatorToPosition(ElevatorState.Handoff),
        Wrist.wristToPosition(WristState.Handoff),
        take.runTakeMotorReverse(1200).until(take.hasCoral),

        take.runTakeMotorReverse(-390).until(take.hasCoral.negate())

    );
  }

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }
}
