// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    //motor and encoder instances
    private final SparkMax m_middlelift;
    private final SparkMax m_innerlift;
    private final RelativeEncoder enc_middlelift;
    private final RelativeEncoder enc_innerlift;

    //pid and ff controllers
    private final SimpleMotorFeedforward ff_middlelift;
    private final ProfiledPIDController pid_middlelift;
    private final SimpleMotorFeedforward ff_innerlift;
    private final ProfiledPIDController pid_innerlift;

    //class-level variables to persist across functions
    private double v_middleliftsp = 0;
    private double v_middleliftpos = 0;
    private double v_middleliftFFout = 0;
    private double v_middleliftPIDout = 0;
    private double v_innerliftsp = 0;
    private double v_innerliftpos = 0;
    private double v_innerliftFFout = 0;
    private double v_innerliftPIDout = 0;

    public Elevator() {

        //declare motors and encoders
        m_middlelift = new SparkMax(ElevatorConstants.k_middleliftID, MotorType.kBrushless);
        m_innerlift = new SparkMax(ElevatorConstants.k_innerliftID, MotorType.kBrushless);
        enc_middlelift = m_middlelift.getEncoder();
        enc_innerlift = m_innerlift.getEncoder();

        //apply configs
        m_middlelift.configure(ElevatorConfigs.cfg_middlelift, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_innerlift.configure(ElevatorConfigs.cfg_innerlift, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //declare FF and PID controllers
        //FFs are required for manual operation to stop elevator and arm from drooping
        //NEED TO CHARACTERIZE FF CONSTANTS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DO NOT USE OTHERWISE BAD THINGS HAPPEN
        ff_middlelift = new SimpleMotorFeedforward(
            ElevatorConstants.k_middleliftkS, 
            ElevatorConstants.k_middleliftkS);
        pid_middlelift = new ProfiledPIDController(
            ElevatorConstants.k_middleliftkP,
            0,
            ElevatorConstants.k_middleliftkD,
            ElevatorConstants.prf_middlelift);
        ff_innerlift = new SimpleMotorFeedforward(
            ElevatorConstants.k_innerliftkS, 
            ElevatorConstants.k_innerliftkV);
        pid_innerlift = new ProfiledPIDController(
            ElevatorConstants.k_innerliftkP, 
            0,
            ElevatorConstants.k_innerliftkD, 
            ElevatorConstants.prf_innerlift);
    }

    /** Function for operating the elevator manually through its feed forwards. Pass speed in units of feet per second!!!!!!!!!
     * 
     * @param vel_middle Desired linear velocity in feet per second for 1st stage.
     * @param vel_inner Desired linear velocity in feet per second for 2nd stage.
     */
    public void manual(double vel_middle, double vel_inner) {

        //temporary to get things moving
        m_middlelift.set(vel_middle);
        m_innerlift.set(vel_inner);

        //calculate next voltage based on velocity, use when characterized
        //v_middleliftFFout = ff_middlelift.calculate(vel_middle);
        //v_innerliftFFout = ff_innerlift.calculate(vel_inner);

        //apply voltage to motors. DO NOT UNCOMMENT UNLESS YOU ARE CERTAIN FF OUTPUT IS RATIONAL!!! USE TELEMETRY TO CHECK!!!!!!
        //m_middlelift.setVoltage(v_middleliftFFout);
        //m_innerlift.setVoltage(v_innerliftFFout);
    }

    //tentative
    public void auto() {

    }

    /**Use this function to change the setpoint of the elevator in auto mode. Can be ran continuously but is good practice to
     * call once and end, as the variable persist in the subsystem.
     * 
     * @param sp_middle Setpoint for 1st stage of elevator.
     * @param sp_inner Setpoint for 2nd stage of elevator.
     */
    public void setpoint(double sp_middle, double sp_inner) {
        v_middleliftsp = sp_middle;
        v_innerliftsp = sp_inner;
    }

    // index legend for data() array
    public static final int middle_lift_sp     = 0;
    public static final int middle_lift_pos    = 1;
    public static final int middle_lift_ffout  = 2;
    public static final int middle_lift_pidout = 3;
    public static final int inner_lift_sp      = 4;
    public static final int inner_lift_pos     = 5;
    public static final int inner_lift_ffout   = 6;
    public static final int inner_lift_pidout  = 7;

    /**
     * Returns a double array with elevator data for troubleshooting or monitoring.
     *
     * <p>Index legend:
     * <ul>
     *   <li>{@link #middle_lift_sp}     – middle lift setpoint</li>
     *   <li>{@link #middle_lift_pos}    – middle lift position</li>
     *   <li>{@link #middle_lift_ffout}  – middle lift feedforward output</li>
     *   <li>{@link #middle_lift_pidout} – middle lift PID output</li>
     *   <li>{@link #inner_lift_sp}      – inner lift setpoint</li>
     *   <li>{@link #inner_lift_pos}     – inner lift position</li>
     *   <li>{@link #inner_lift_ffout}   – inner lift feedforward output</li>
     *   <li>{@link #inner_lift_pidout}  – inner lift PID output</li>
     * </ul>
     *
     * @return array of elevator values in the order described above
     */
    public double[] data() {
        return new double[] {
            v_middleliftsp,
            v_middleliftpos,
            v_middleliftFFout,
            v_middleliftPIDout,
            v_innerliftsp,
            v_innerliftpos,
            v_innerliftFFout,
            v_innerliftPIDout
        };
    }

    //put periodic things here
    @Override
    public void periodic() {

        //update position data
        v_middleliftpos = enc_middlelift.getPosition();
        v_innerliftpos = enc_innerlift.getPosition();
    }
}
