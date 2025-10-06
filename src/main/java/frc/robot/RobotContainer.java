// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.MMotorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    //subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Elevator m_Elevator = new Elevator();
    private final Arm m_Arm = new Arm();

    //chooser for autos for PP
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    //used to toggle robot oriented mode, passed through default command for drive subsystem
    private boolean robotOriented;

    //controllers
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        //schedules periodic blocks in subsystems
        m_robotDrive.register();
        m_Elevator.register();
        m_Arm.register();

        m_robotDrive.configureAutobuilder();

        //put named commands here
        //example
        //NamedCommands.registerCommand("test", null);

        autoChooser.addOption("Auto A", AutoBuilder.buildAuto("Auto A"));
        autoChooser.addOption("Auto B", AutoBuilder.buildAuto("Auto B"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();

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

        robotOriented = true;

        //operate elevator in manual by default, uses controller inputs for now
        //CHECK FOR INVERSIONS!!!!!!!!!!!!!!!!!!!!
        m_Elevator.setDefaultCommand(
            new RunCommand(
                () -> m_Elevator.manual(
                    (m_operatorController.leftTrigger().getAsBoolean() ? 0.5 : 0) + (m_operatorController.leftBumper().getAsBoolean() ? -0.5 : 0), 
                    (m_operatorController.getLeftY() < -0.5 ? 0.5 : 0) + (m_operatorController.getLeftY() > 0.5 ? -0.5 : 0)), 
                m_Elevator));
        
        //operate arm in manual by default in teleop
        m_Arm.setDefaultCommand(
            new RunCommand(
                () -> m_Arm.arm_manual(
                    (m_operatorController.rightTrigger().getAsBoolean() ? 0.5 : 0) + (m_operatorController.rightBumper().getAsBoolean() ? -0.5 : 0)),
                m_Arm));
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
        
        //hold for robot oriented, use other line if toggle is desired
        m_driverController.y().whileTrue(runEnd(() -> robotOriented = false, () -> robotOriented = true));
        //m_driverController.y().onTrue(runOnce(() -> robotOriented = !robotOriented));
    }



    //primitive autos
    public Command driveForward() {
        return run(() -> m_robotDrive.drive(-0.5, 0, 0, false), m_robotDrive).withTimeout(1.5);
    }

    public Command driveBackward() {
        return run(() -> m_robotDrive.drive(0.1, 0, 0, false), m_robotDrive).withTimeout(.25);
    }



    //use to pass the autonomous command to robot framework
    public Command getAutonomousCommand() {
        //return autoChooser.getSelected();
        //The above line(pathplanner integration) won't work until we rewrite all subsystems to command, which is necessary for pathplanner
        
        return this.driveForward();//.andThen(this.driveBackward().withTimeout(.25)).andThen(this.raisePivot()); 
        
        //return this.driveBackward().andThen(this.raisePivot());
        //return null;
    }


    
    //self explanatory, add more data as needed to troubleshoot and such
    public void telemetry() {
        SmartDashboard.putNumber("drive controller right X", m_driverController.getRightX());
        SmartDashboard.putNumber("drive controller left X", m_driverController.getLeftX());
        SmartDashboard.putNumber("drive controller left Y", m_driverController.getLeftY());
        SmartDashboard.putNumber("drive heading", m_robotDrive.getHeading());
        SmartDashboard.putNumber("arm encoder", m_Arm.data()[1]);
    }
}