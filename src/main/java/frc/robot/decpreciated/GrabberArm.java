// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.decpreciated;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberArm extends SubsystemBase {

    private final SparkMax m_grabberJoint;

    private final AbsoluteEncoder enc_grabber;


    private final TrapezoidProfile.Constraints con_armconstraints = new TrapezoidProfile.Constraints(0.5*Math.PI, 0.25*Math.PI);
    private final ProfiledPIDController pid_armcontroller = new ProfiledPIDController(0.1, 0, 0, con_armconstraints);
    private final ArmFeedforward ff_armcontroller = new ArmFeedforward(0, 0.1, 2.5235);

    private double var_encoderposition;
    private double var_pidoutput;
    private double var_ffoutput;
    private static final double k_encoderoffset = 1.2 + Math.PI;


  public GrabberArm(SparkMax sparkmax) {
    m_grabberJoint = sparkmax;
    enc_grabber = m_grabberJoint.getAbsoluteEncoder();
    pid_armcontroller.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void closedLoop(double setpoint) {
    pid_armcontroller.setGoal(setpoint);
    var_pidoutput = pid_armcontroller.calculate(var_encoderposition);
    var_ffoutput = ff_armcontroller.calculate(setpoint, pid_armcontroller.getSetpoint().velocity);
  }

  public double getPosition() {
    return var_encoderposition;
  }

  public double[] getPIDdata() {
    return new double[] {
      var_pidoutput,
      var_ffoutput
    };
  }

  @Override
  public void periodic() {
    var_encoderposition = MathUtil.angleModulus(enc_grabber.getPosition() - k_encoderoffset);
  }
}
