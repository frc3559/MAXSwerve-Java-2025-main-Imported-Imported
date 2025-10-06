// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ArmConfigs;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    //motors 'n encoders
    private final SparkMax m_arm;
    private final SparkMax m_grabber;
    private final AbsoluteEncoder enc_arm;

    //FF and PID
    private final SimpleMotorFeedforward ff_arm;
    private final ProfiledPIDController pid_arm;

    //persistenet variables for sending telemetry
    private double v_armsp = 0;
    private double v_armpos = 0;
    private double v_armFFout = 0;
    private double v_armPIDout = 0;
    private double v_grabberspeed = 0;

    public Arm() {
        
        //motor and encoder declarations
        m_arm = new SparkMax(ArmConstants.k_armID, MotorType.kBrushless);
        m_grabber = new SparkMax(ArmConstants.k_grabberID, MotorType.kBrushless);
        enc_arm = m_arm.getAbsoluteEncoder();

        //configure spark maxeeeeeeeeeeeeeeeeeeeeessssss
        m_arm.configure(ArmConfigs.cfg_arm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_grabber.configure(ArmConfigs.cfg_grabber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //ff and pid
        ff_arm = new SimpleMotorFeedforward(
            ArmConstants.k_armkS,
            ArmConstants.k_armkV);
        pid_arm = new ProfiledPIDController(
            ArmConstants.k_armkP,
            0,
            ArmConstants.k_armkD,
            ArmConstants.prf_arm);
    }

    //tentative
    public void arm_auto() {

    }
    
    /**Operate arm manually using feed fowards. 
     * 
     * @param vel_arm Velocity of arm in rads/s.
     */
    public void arm_manual(double vel_arm) {

        //temporary to get function going
        m_arm.set(vel_arm);

        //calculate FF for arm and send
        v_armFFout = ff_arm.calculate(vel_arm);

        //do not uncomment until FF output is rational
        //m_arm.setVoltage(v_armFFout);
    }

    /**Function to send setpoint to the arm when in auto mode. 
     * 
     * @param arm_sp Setpoint angle in radians.
     */
    public void setpoint(double arm_sp) {

        //pass setpoint into subsystem
        v_armsp = arm_sp;
    }

    /**Run grabber motor with passed speed.
     * 
     * @param vel_grabber Motor speed in units of volts.
     */
    public void grabber(double vel_grabber) {

        //start grabber up with passed speed
        v_grabberspeed = vel_grabber;
        m_grabber.setVoltage(v_grabberspeed);
    }

    // index legend for data() array
    public static final int arm_sp      = 0;
    public static final int arm_pos     = 1;
    public static final int arm_ffout   = 2;
    public static final int arm_pidout  = 3;
    public static final int grabber_spd = 4;

    /**
     * Returns a double array with elevator data for troubleshooting or monitoring.
     *
     * <p>Index legend:
     * <ul>
     *   <li>{@link #arm_sp}      – arm setpoint</li>
     *   <li>{@link #arm_pos}     – arm position</li>
     *   <li>{@link #arm_ffout}   – arm feedforward output</li>
     *   <li>{@link #arm_pidout}  – arm lift PID output</li>
     *   <li>{@link #grabber_spd} – grabber speed</li>
     * </ul>
     *
     * @return array of elevator values in the order described above
     */
    public double[] data() {
        return new double[] {
            v_armsp,
            v_armpos,
            v_armFFout,
            v_armPIDout,
            v_grabberspeed
        };
    }

    @Override
    public void periodic() {

        //update arm position
        v_armpos = enc_arm.getPosition();
    }
}
