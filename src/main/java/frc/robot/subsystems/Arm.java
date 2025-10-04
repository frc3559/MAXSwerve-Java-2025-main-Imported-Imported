// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    //motors
    private final SparkMax m_arm;
    private final SparkMax m_grabber;

    //encoder
    private final AbsoluteEncoder enc_arm;

    public Arm() {
        
        //motor declarations
        m_arm = new SparkMax(ArmConstants.k_armID, MotorType.kBrushless);
        m_grabber = new SparkMax(ArmConstants.k_grabberID, MotorType.kBrushless);

        //encoder
        enc_arm = m_arm.getAbsoluteEncoder();
    }

    public void manual() {

    }

    @Override
    public void periodic() {
    }
}
