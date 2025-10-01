// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runEnd;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.MMotorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberArm;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    //sendableChosoer
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  //spark maxes
  private final SparkMax m_lock = new SparkMax(20, MotorType.kBrushless);
  private final SparkMax m_lift = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax m_grabberWheel = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax m_grabberJoint = new SparkMax(24, MotorType.kBrushless);


  //encoders for lock
  private final RelativeEncoder m_lockencEncoder = m_lock.getEncoder();

  //subsystem for grabber
  private final GrabberArm m_grabber = new GrabberArm(m_grabberJoint);

  //timer for basic auto
  private Timer autoTimer = new Timer();

  //setpoint for grabber
  private double grabber_angle = 0;

  private boolean robotOriented;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive.configureAutobuilder();

    //put named commands here
    //example
    //NamedCommands.registerCommand("test", null);

    autoChooser.addOption("Auto A", AutoBuilder.buildAuto("Auto A"));
    autoChooser.addOption("Auto B", AutoBuilder.buildAuto("Auto B"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
   
    // Configure the button bindings
    configureButtonBindings();

    m_grabberJoint.configure(Configs.MAXSwerveModule.grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*MMotorConstants.kDriveSpeedModifier, MMotorConstants.kDriveDeadband)*MMotorConstants.kDriveSpeedModifier,
                -MathUtil.applyDeadband(m_driverController.getLeftX()*MMotorConstants.kDriveSpeedModifier, MMotorConstants.kDriveDeadband)*MMotorConstants.kDriveSpeedModifier,
                -MathUtil.applyDeadband(m_driverController.getRawAxis(4)*MMotorConstants.kDriveSpeedModifier, MMotorConstants.kDriveDeadband)*MMotorConstants.kDriveSpeedModifier,
                robotOriented),
            m_robotDrive));

            m_lockencEncoder.setPosition(0);

            robotOriented = true;

    m_grabber.setDefaultCommand(run(() -> m_grabber.closedLoop(grabber_angle), m_grabber));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //robot oriented mode switch, robot oriented is activated normally, invert if field oriented is desired normally
    m_driverController.y().onTrue(runOnce(() -> robotOriented = false)).onFalse(runOnce(() -> robotOriented = true));

    //run lock motor until encoder reports > limit
    m_operatorController.povLeft().whileTrue(runEnd(() -> m_lock.set(-0.1), () -> m_lock.set(0)).withTimeout(0.2));
    m_operatorController.povRight().whileTrue(runEnd(() -> m_lock.set(0.1), () -> m_lock.set(0)).withTimeout(0.2));

    //run lift up and down
    m_operatorController.povUp().whileTrue(runEnd(() -> m_lift.set(1), () -> m_lift.set(0)));
    m_operatorController.povDown().whileTrue(runEnd(() -> m_lift.set(-1), () -> m_lift.set(0)));

    //run grabber claw intake/shoot
    m_operatorController.leftTrigger().whileTrue(runEnd(() -> m_grabberWheel.set(-MMotorConstants.kGrabberWheelInSpeed), () -> m_grabberWheel.set(0)));
    m_operatorController.rightTrigger().whileTrue(runEnd(() -> m_grabberWheel.set(MMotorConstants.kGrabberWheelOutSpeed), () -> m_grabberWheel.set(0)));
    
    //run grabber joint
    m_operatorController.a().whileTrue(runEnd(() -> m_grabberJoint.set(MMotorConstants.kGrabberJointSpeed), () -> m_grabberJoint.set(0)));
    m_operatorController.b().whileTrue(runEnd(() -> m_grabberJoint.set(-MMotorConstants.kGrabberJointSpeed), () -> m_grabberJoint.set(0)));

  }

  public Command driveForward() {
    return run(() -> m_robotDrive.drive(-0.5, 0, 0, false), m_robotDrive).withTimeout(1.5);
  }

  public Command driveBackward() {
    return run(() -> m_robotDrive.drive(0.1, 0, 0, false), m_robotDrive).withTimeout(.25);
  }

  public Command raisePivot(){
    double timeout = 1.4;
    double speed = 0.3;
    return run(() -> m_grabberJoint.set(speed)).withTimeout(timeout).andThen(runOnce(() -> this.m_grabberJoint.set(0)));
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    //The above line(pathplanner integration) won't work until we rewrite all subsystems to command, which is necessary for pathplanner
    
    return this.driveForward();//.andThen(this.driveBackward().withTimeout(.25)).andThen(this.raisePivot()); 
    
    //return this.driveBackward().andThen(this.raisePivot());
    //return null;
  }

  public void telemetry() {
    SmartDashboard.putNumber("drive controller right X", m_driverController.getRightX());
    SmartDashboard.putNumber("drive controller left X", m_driverController.getLeftX());
    SmartDashboard.putNumber("drive controller left Y", m_driverController.getLeftY());
    SmartDashboard.putNumber("drive heading", m_robotDrive.getHeading());
    SmartDashboard.putNumber("Grabber arm absolute position", m_grabber.getPosition());
    SmartDashboard.putNumber("grabber PID output", m_grabber.getPIDdata()[0]);
    SmartDashboard.putNumber("grabber FF output", m_grabber.getPIDdata()[1]);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   /**
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
    */
    
}