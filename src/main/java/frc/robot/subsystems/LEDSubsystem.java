package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LEDSubsystem extends SubsystemBase{

    private double colorValue;
    public Spark m_Spark;
    public Spark m_Spark2;

    public LEDSubsystem() {

    //Create BlinkIn
    m_Spark = new Spark(0);
    m_Spark2 = new Spark(1);

    colorValue = 0.61;

    // 0.87 - Blue Team
    // 0.61 - Red Team
    
    }

    public void lightTest(){
        colorValue = -0.83; //Shot, Blue
    }

    public void lightTest2(){ //Shot, White
        colorValue = -0.81;
    }

    public void lightTest3(){ //Solid Blue
        colorValue = 0.87;
    }

    public void lightTest4(){ //Solid White
        colorValue = 0.93;
    }

    public void lightTest5(){ //Solid Yellow
        colorValue = 0.69;
    }

    public void lightTest6(){ //Solid Purple
        colorValue = 0.91;
    }

    public void lightTest7(){
        colorValue = -0.87;
    }

    public void lightTest8(){ //Light Chase, Blue
        colorValue = -0.29;
    }

    public void autoStart(){ //Lime
        colorValue = 0.73;
    }

    public void disableInit(){
        m_Spark.disable();
        m_Spark2.disable();
    }

    public void resetAll(){ 
        colorValue = 0;
    }

    @Override
    public void periodic(){
        m_Spark.set(colorValue);
        m_Spark2.set(colorValue);

    }
}
