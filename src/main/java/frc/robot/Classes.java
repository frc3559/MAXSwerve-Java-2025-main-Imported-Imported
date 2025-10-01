//You're welcome!
package frc.robot;
//As of now, totally, laughably broken. Seriously. I hate Java and I hope whoever made it is suffering.
//imports here

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MMotorConstants;
import frc.robot.Constants.OIConstants;

//STOP IMPORTING HERE (or else)
/**
public class Classes {
    // Operator Controller
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    //Motor declarations -- arm
  SparkMax armJoint = new SparkMax(DriveConstants.kArmJointCanId, MotorType.kBrushless);
  SparkMax armWheel = new SparkMax(DriveConstants.kArmWheelCanId, MotorType.kBrushless);
  //Motor declarations -- grabber
  //SparkMax grabberWheel = new SparkMax(DriveConstants.kGrabberWheelCanId, MotorType.kBrushless);
  //Limit Switch Declarations
  //DigitalInput grabberSwitch = new DigitalInput(0);


    public void runArmWheel() {
        if (m_operatorController.getAButton()) {
            armWheel.set(MMotorConstants.kArmWheelSpeedModifier);
        } else {
            if (m_operatorController.getBButton()) {
                armWheel.set(-MMotorConstants.kArmWheelSpeedModifier);
            } else {
                armWheel.set(0);
            }
        }
    }
    public static void armWheelRun() {
        Classes run = new Classes();
        run.runArmWheel();
    }
}
    */