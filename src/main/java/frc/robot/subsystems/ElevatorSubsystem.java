package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State.*;
import edu.wpi.first.wpilibj.DigitalInput;


public class ElevatorSubsystem extends SubsystemBase {
    public HState hState;
    public VState vState;

    private final TalonFX m_ExtendMotor;
    public final TalonFX m_RiseMotor;

    private DigitalInput s_RiseSwitch;
    private DigitalInput s_ExtendSwitch;

    private boolean s_ExtendState;
    private boolean s_RiseState;

    private boolean extendEnable;
    private boolean riseEnable;

    // PID Controllers
    private PIDController hPID;
    private PIDController vPID;
    private ElevatorFeedforward vFeedforward;
    
    // PID setpoints
    private double hSetPoint;
    private double vSetPoint;

    // Pose Parameters
    private double hHome;
    private double hLoadLow;
    private double hLoadCone;
    private double hRampCube;
    private double hRampCone;
    private double hScoreCubeLow;
    private double hScoreCubeHigh;
    private double hScoreConeHigh;
    private double hScoreConeLow;
    private double hMax;

    private double hTest;
    
    private double vHome;
    private double vLoadLow;
    private double vLoadCone;
    private double vRampCube;
    private double vRampCone;
    private double vScoreCubeLow;
    private double vScoreConeLow;
    private double vScoreCubeHigh;
    private double vScoreConeHigh;
    private double vMax;

    //Driver Movement
    private double M_RISE;

    private double m_FORWARD;
    private double m_REVERSE;

    //private double VSpeed;

    public ElevatorSubsystem() {
         
        //100,000 / 3.5 in

        extendEnable = false;
        riseEnable = true;

        //Elevator Horizontal Shift / EXTENSION
        m_ExtendMotor = new TalonFX(Constants.Elevator.ElevatorExtend.extendMotorID);
        hHome =  m_ExtendMotor.getSelectedSensorPosition();  // zero
        s_ExtendSwitch = new DigitalInput(0);

        //Elevator Vertical Shift / ELEVATION
        m_RiseMotor = new TalonFX(Constants.Elevator.ElevatorRise.ElevatorMotorID);
        vHome = m_RiseMotor.getSelectedSensorPosition();  // zero
        s_RiseSwitch = new DigitalInput(1);

        // Parameters
        double hP = 0.00015; // was 0.0015
        double hI = 0.000010; // was 0.0000025, 0.0000035, 0.00025
        double hD = 0.000065;  // was 0.00475, 0.00500
        hPID = new PIDController(hP, hI, hD);

        double vP = 0.00005; // was 0.0005
        double vI = 0; // was 0.000100
        double vD = 0;  // was 0.00475
        vPID = new PIDController(vP, vI, vD);

        vFeedforward = new ElevatorFeedforward(0, 0.085, 0);

        // Set Points
        vSetPoint = vHome;
        hSetPoint = hHome;

        //HORIZONTAL
        //hHome = -110000; //MIN
        hLoadLow = hHome + 1000; // guess
        hLoadCone = hHome; // guess
        hRampCube = 0;       // guess
        hRampCone = hHome + 80000;       // guess
        hScoreCubeLow = hHome + 400000;   // guess
        hScoreConeLow = hHome + 75000;   // guess
        hScoreCubeHigh = hHome + 400000;  // guess, hHome + 329411
        hScoreConeHigh = hHome + 400000;  // guess
        //hMax = 400000; //MAX

        //VERTICAL
        //vHome = 0; //MIN
        vLoadLow = vHome + 40000; //guess
        vLoadCone = vHome + 24863; //guess
        vRampCube = vHome + 1;       // guess
        vRampCone = vHome;       // guess
        vScoreCubeLow = vHome;   // guess
        vScoreConeLow = vHome + 73597;   // guess
        vScoreCubeHigh = vHome + 75684;  // guess
        vScoreConeHigh = vHome + 50000;  // guess
        //vMax = 81490; //MAX

        //Test Point
        hTest = -56792;

        //Driver Movement
        m_FORWARD = hHome + 80000;
        m_REVERSE = hHome + 1000;

        hState = HState.HOME;
        vState = VState.HOME;
    }

    private double hPos() {
        // get the horizontal position
        return m_ExtendMotor.getSelectedSensorPosition();
    }

    private double vPos() {
        // get the vertical position value
        return m_RiseMotor.getSelectedSensorPosition();
    }

    private void setHPos(double x) {
        // set the horizontal demand position "set point"
        hSetPoint = x;
    }
    
    private void setVPos(double y) {
        // set the vertical demand position "set point"
        vSetPoint = y;
    }

    private boolean hBackLimit() {
        // check if the "home" limit switch for the horizontal carriage is activated
        return true;  // return true until button is real
    }

    private void zeroHCarriage() {
        // An algorithm to set the horiontal carriage to its home position using the limit switch.
        // 1. move the carriage (if in the home position move minimally to un home)
        // 2. move the carriage backwards slowly
        // 3. when limit switch is hit stop the motor and zero the encoder
        // nudge forward
        setHPos(3);
        while(!hBackLimit()) {
            m_ExtendMotor.set(ControlMode.PercentOutput, -0.1);
        }
        m_ExtendMotor.setSelectedSensorPosition(0);
        setHPos(0);
    }

    private HState getHState() {
        // get the current value of the horizontal carriage, returns a value in HState
        // if no define value is determined, return the previous state. 
        // that is when the carriage is moving between defined points the state should be treated as the previous
        // until the move ends
        double x = hPos();
        if (x == vHome) {
            return HState.HOME;
        } else if (x == vLoadLow) {
            return HState.LOAD_LOW;
        } else if (x == vLoadCone) {
            return HState.LOAD_CONE;
        } else if (x == vRampCube) {
            return HState.RAMP_CUBE;
        } else if (x == vRampCone) {
            return HState.RAMP_CONE;
        } else if (x == vScoreCubeLow) {
            return HState.SCORE_CUBE_LOW;
        } else if (x == vScoreConeLow) {
            return HState.SCORE_CONE_LOW;
        } else if (x == vScoreCubeHigh) {
            return HState.SCORE_CUBE_HIGH;
        } else if (x == vScoreConeHigh) {
            return HState.SCORE_CONE_HIGH;
        }
        return hState;
    }

    private VState getVState() {
        // get the current value of the vertical lift, returns a value in VState
        // if no define value is determined, return the previous state. 
        // that is when the lift is moving between defined points the state should be treated as the previous
        // until the move ends
        double y = vPos();
        if (y == vHome) {
            return VState.HOME;
        } else if (y == vLoadLow) {
            return VState.LOAD_LOW;
        } else if (y == vLoadCone) {
            return VState.LOAD_CONE;
        } else if (y == vRampCube) {
            return VState.RAMP_CUBE;
        } else if (y == vRampCone) {
            return VState.RAMP_CONE;
        } else if (y == vScoreCubeLow) {
            return VState.SCORE_CUBE_LOW;
        } else if (y == vScoreConeLow) {
            return VState.SCORE_CONE_LOW;
        } else if (y == vScoreCubeHigh) {
            return VState.SCORE_CUBE_HIGH;
        } else if (y == vScoreConeHigh) {
            return VState.SCORE_CONE_HIGH;
        }
        return vState;
    }

    private void HStateSwitch(){
        
    }

    private void VStateSwitch(){
        
    }

    //Driver Custom Movement

    public void carriageExtend(){
        setHPos(m_FORWARD);
    }

    public void carriageRetract(){
        setHPos(m_REVERSE);
    }

    public void carriageStop(){
        setHPos(hPos());
    }

   public void elevatorUp(){
        M_RISE = 0.25; // or -0.25. depends on polarity
        //mRiseMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void elevatorDown(){
        M_RISE = -0.25; 
        //mRiseMotor.setNeutralMode(NeutralMode.Brake);

    }

    //Hold Elevator Motor
    public void stopElevator(){
        M_RISE = 0.10;
    }

    public void resetAll(){
        M_RISE = 0; 
    }

    @Override
    public void periodic() {

        double hPV = hPos();
        double hOutput = hPID.calculate(hPV, hSetPoint); // compute H PID control value

        double vPV = vPos();
        double vOutput = vPID.calculate(vPV, vSetPoint) + vFeedforward.calculate(vSetPoint); // compute V PID control value

        //FAILSAFE, If going past home position
        if(hSetPoint < hHome || hPV < hHome){
            m_ExtendMotor.set(ControlMode.PercentOutput, 0);
        }

       if(extendEnable) {
            m_ExtendMotor.set(ControlMode.PercentOutput, hOutput);
            m_ExtendMotor.setNeutralMode(NeutralMode.Brake);
        }

        if (riseEnable) {
            m_RiseMotor.set(ControlMode.PercentOutput, M_RISE); //MathUtil.clamp(vOutput, -1, 1)
            m_RiseMotor.setNeutralMode(NeutralMode.Brake);
        }

        //Update States
        getHState();
        getVState();

        // update telemetry
        SmartDashboard.putNumber("Horizontal Encoder", hPV);
        SmartDashboard.putNumber("Horizontal Set Point", hSetPoint);
        SmartDashboard.putNumber("Horizontal Output", hOutput);
        
        SmartDashboard.putNumber("Vertical Encoder", vPV);
        SmartDashboard.putNumber("Vertical Set Point", vSetPoint);
        SmartDashboard.putNumber("Vertical Output", vOutput);

        SmartDashboard.putBoolean("Horizontal Enable", extendEnable);
        SmartDashboard.putBoolean("Vertical Enable", riseEnable);

        SmartDashboard.putString("Horizontal Mode", hState.name());
        SmartDashboard.putString("Vertical Mode", vState.name());

        SmartDashboard.putBoolean("Horizontal Switch", s_ExtendState);
        SmartDashboard.putBoolean("Vertical Switch", s_RiseState);
    }
    // Public Functions to command the subsystem into the consituent pose positions
    // they use the setHPos and setVPos methods with the h___ and v___ constans defined in the constructor
    // to command motion using the HState and VState position language
    public void goHState(HState pos) {
        if (pos == HState.HOME) {
            setHPos(vHome);
            hState = HState.HOME;
        
        } else if (pos == HState.LOAD_LOW) {
            setHPos(hLoadLow);
            hState = HState.LOAD_LOW;
        
        } else if (pos == HState.LOAD_CONE) {
            setHPos(hLoadCone);
            hState = HState.LOAD_CONE;
        
        } else if (pos == HState.RAMP_CUBE) {
            setHPos(hRampCube);
            hState = HState.RAMP_CUBE;
        
        } else if (pos == HState.RAMP_CONE) {
            setHPos(hRampCone);
            hState = HState.RAMP_CONE;
        
        } else if (pos == HState.SCORE_CUBE_LOW) {
            setHPos(hScoreCubeLow);
            hState = HState.SCORE_CUBE_LOW;
       
        } else if (pos == HState.SCORE_CONE_LOW) {
            setHPos(hScoreConeLow);
            hState = HState.SCORE_CONE_LOW;
        
        } else if (pos == HState.SCORE_CUBE_HIGH) {
            setHPos(hScoreCubeHigh);
            hState = HState.SCORE_CUBE_HIGH;
        
        } else if (pos == HState.SCORE_CONE_HIGH) {
            setHPos(hScoreConeHigh);
            hState = HState.SCORE_CONE_HIGH;
        }
    }

    public void goVState(VState pos) {
        if (pos == VState.HOME) {
            //setVPos(vHome);
            vState = VState.HOME;
        
        } else if (pos == VState.LOAD_LOW) {
            setVPos(vLoadLow);
            vState = VState.LOAD_LOW;
        
        } else if (pos == VState.LOAD_CONE) {
            setVPos(vLoadCone);
            vState = VState.LOAD_CONE;
        
        } else if (pos == VState.RAMP_CUBE) {
            setVPos(vRampCube);
            vState = VState.RAMP_CUBE;
        
        } else if (pos == VState.RAMP_CONE) {
            setVPos(vRampCone);
            vState = VState.RAMP_CONE;
       
        } else if (pos == VState.SCORE_CUBE_LOW) {
            setVPos(vScoreCubeLow);
            vState = VState.SCORE_CUBE_LOW;
        
        } else if (pos == VState.SCORE_CONE_LOW) {
            setVPos(vScoreConeLow);
            vState = VState.SCORE_CONE_LOW;
        
        } else if (pos == VState.SCORE_CUBE_HIGH) {
            setVPos(vScoreCubeHigh);
            vState = VState.SCORE_CUBE_HIGH;
        
        } else if (pos == VState.SCORE_CONE_HIGH) {
            setVPos(hTest);
            vState = VState.SCORE_CONE_HIGH;
        }
    }
    
    public void disableMotors() {
        extendEnable = false;
        riseEnable = false;
    }
    public void enableMotors() {
        extendEnable = true;
        riseEnable = true;
    }
}
//Max value: 81490.000000
