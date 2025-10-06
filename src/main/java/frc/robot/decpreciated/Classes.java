//You're welcome!
package frc.robot.decpreciated;
//As of now, totally, laughably broken. Seriously. I hate Java and I hope whoever made it is suffering.
//i blame james gosling
//imports here

//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
//import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.MMotorConstants;
//import frc.robot.Constants.OIConstants;

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






    //this would kind of work if you started it in the robot container, but some of the instance references will not work. 
    //All object instances must either be started by a parent or child inside of the "Main" framework. 
    //Further explanation of how the framework work can be provided if needed

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








    //you are attempting to start an instance of the parent class inside of the child/parent, this cannot work in this way
    //essentially the java version of an infinite russian nesting doll
    //again class instances must be ran inside of the robot / main framework as mentioned above

    public static void armWheelRun() {
        Classes run = new Classes();
        run.runArmWheel();
    }
}
    */