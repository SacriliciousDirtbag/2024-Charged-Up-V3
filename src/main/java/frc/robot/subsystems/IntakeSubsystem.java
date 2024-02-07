package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.math.Conversions;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class IntakeSubsystem extends SubsystemBase {
    /*Data Types*/
    
    public FState fState;  // flipper state
    public SState sState;  // spinner state
    
    //OBJECTS
    private final TalonFX m_FlipMotor;
    private final TalonFX m_SpinMotor;

    private DutyCycleEncoder f_Encoder;

    private boolean flipEnable;
    private boolean spinEnable;

    // PID Controller
    private PIDController fPID;
    private PIDController fPID2;
    private ArmFeedforward fFeedforward;
    private ArmFeedforward fFeedforward2;

    // PID set point
    private double fSetPoint;
    private double sSetPoint;

    // Pose Parameters
    private double fHome;
    private double fLoadLow;
    private double fLoadCone;
    private double fRampCube;
    private double fRampCone;
    private double fScoreCubeLow;
    private double fScoreCubeHigh;
    private double fScoreConeLow;
    private double fScoreConeHigh;
    private double fFlipped;
    
    private double sIn;
    private double sOut;
    private double sStop;

    private double spinCurrentLimit;

    //Driver Movement
    private double m_UP;
    private double m_DOWN;

    private double M_INTARGET;

    private boolean fGB;

    public IntakeSubsystem() {

        //WHEELS
        sIn = 0.85;
        sOut = -0.85;
        sStop = 0;

        spinCurrentLimit = 25; // guess

        flipEnable = true;
        spinEnable = false;

        //DUTY CYCLE ENCODER
        f_Encoder = new DutyCycleEncoder(3);
        f_Encoder.reset();
        f_Encoder.setPositionOffset(0);
        //f_Encoder.setDistancePerRotation(2048);

        

        //Flip Intake - TALON / FLIP ARM
        m_FlipMotor = new TalonFX(Constants.IntakeSystem.IntakeFlip.flipMotorID);
        m_FlipMotor.configFactoryDefault();

        //Spin Wheels - TALON / SPIN WHEELS
        m_SpinMotor = new TalonFX(Constants.IntakeSystem.IntakeWheel.inMotorID);
        m_SpinMotor.configFactoryDefault();
        sStop = m_SpinMotor.getSelectedSensorPosition();        

        double fP = 0.005; //0.0002515, 0.00015, 0.000615, 0.000075
        double fI = 0.0; //0.00000585, 0.00000575, 0.00000575
        double fD = 0.15; //0.00000108, 0.0001
        fPID = new PIDController(fP, fI, fD);

        //UP PID
        double ffP = 0.000200; //0.0002515, 0.00015, 0.000615
        double ffI = 0.00000575; //0.00000585, 0.00000575
        double ffD = 0.00000108; //0.00000108
        fPID2 = new PIDController(ffP, ffI, ffD);

        fFeedforward = new ArmFeedforward(0, 0.2, 0); //-0.15
        fFeedforward2 = new ArmFeedforward(0, 0.2, 0);
        
        // Set Points
        fSetPoint = 355;
        sSetPoint = sStop;

        //INTAKE
        fHome = 355;
        fLoadLow = 190; 
        fLoadCone = fHome - 1; //Intake Position
        fRampCube = 260; // guess
        fRampCone = fHome - 20; // guess
        fScoreCubeLow = fHome - 20; // guess, 15000
        fScoreConeLow = fHome - 20; // guess
        fScoreCubeHigh =  285; // guess, fHome - 23000;
        fScoreConeHigh = fHome - 23; // guess
        fFlipped = fHome; //MAX
        //fhome - 30000 is Gravity Barrier

        //Driver Movement
        m_UP = 1;
        m_DOWN = 1;

        fState = FState.HOME;
        sState = SState.STOP;
    }

    private double fPos() {
        return f_Encoder.getAbsolutePosition() * 360;
    }

    private double sPos() {
        return m_SpinMotor.getSelectedSensorPosition();
    }

    private double sCurrent() {
        return m_SpinMotor.getStatorCurrent();
    }

    private void setFPos(double f) {
        fSetPoint = f;
    }

    private void setSPos(double s) {
        sSetPoint = s;
    }

    private FState getFState() {
        double f = fPos();
        if (f == fHome) {
            return FState.HOME;
        } else if (f == fRampCube) {
            return FState.RAMP_CUBE;
        } else if (f == fRampCone) {
            return FState.RAMP_CONE;
        }
        return fState;
    }

    private SState getSState() {
        double s = sPos();
        if (s == sStop) {
            return SState.HOME;
        } else if (s == sIn) {
            return SState.IN;
        } else if (s == sOut) {
            return SState.OUT;
        }
        return sState;
    }

    //Driver Custom Movement

    /*public void flipForward(){
        setFPos(fPos() + m_UP);
    }

    public void flipReverse(){
        if(fPos() != fHome || fPos() > fHome){
            setFPos(fPos() - m_DOWN);
        }
    }*/

    public void flipStop(){
        setFPos(fPos());
    }

    //FORWARD wheel spin
    public void wheelForward(){
        M_INTARGET = 1; 
    }
    
    //REVERSE wheel spin
    public void wheelReverse(){
        M_INTARGET = -1; //was -0.8
    }

    public void holdWheels(){
        M_INTARGET = 0.05;
    }

    public void holdWheelsNeg(){
        M_INTARGET = -0.05;  
    }

    //RESET motors & wheels
    public void resetAll(){
        M_INTARGET = 0;
    }

    @Override
    public void periodic(){
        double fPV = fPos();
        double fOutput = fPID.calculate(fPV, fSetPoint);

        //FAILSAFE, If going past home position
        /*if(fSetPoint > fHome || fPV >= fHome && fState == FState.FLIPPED){
            m_FlipMotor.set(ControlMode.PercentOutput, 0);
        }*/
        
        if (flipEnable) { //Under Gravity Barrier -> Arbitrary Feedforward
            if(fPV == fHome && fState == FState.FLIPPED) {
                fOutput = fPID.calculate(fPV, fSetPoint);
                m_FlipMotor.set(ControlMode.PercentOutput, 0.25);
                fGB = true;
            } else {
                fOutput = fPID.calculate(fPV, fSetPoint);
                m_FlipMotor.set(ControlMode.PercentOutput, MathUtil.clamp(fOutput, -0.4, 0.4));
                m_FlipMotor.setNeutralMode(NeutralMode.Brake);
                fGB = false;
            }

            //m_FlipMotor.set(ControlMode.PercentOutput, fOutput + fFeedforward.calculate(fSetPoint, fOutput)); //consider using positon control mode, need PID + sensor pos
        }

        m_SpinMotor.set(ControlMode.PercentOutput, M_INTARGET);
        //m_SpinMotor.setNeutralMode(NeutralMode.Brake);

        if (sCurrent() > spinCurrentLimit) {
            spinEnable = false;
            sState = SState.STOP;
        }

        fState = getFState();
        sState = getSState();

        SmartDashboard.putNumber("Flip Encoder", fPV);
        SmartDashboard.putNumber("Flip Set Point", fSetPoint);
        SmartDashboard.putNumber("Flip Output", fOutput);

        SmartDashboard.putNumber("Flip Velocity", m_FlipMotor.getSelectedSensorVelocity());
        
        SmartDashboard.putNumber("Spin Current", m_SpinMotor.getStatorCurrent());
        SmartDashboard.putNumber("Spin Set Point", sSetPoint);

        SmartDashboard.putBoolean("Flip Enable", flipEnable);
        SmartDashboard.putString("Flip Mode", fState.name());

        SmartDashboard.putBoolean("Gravity Barrier", fGB);

        //DCE
        SmartDashboard.putNumber("DC Encoder", fPV);
        SmartDashboard.putNumber("DC Offset", f_Encoder.getPositionOffset());

        SmartDashboard.putBoolean("DCE Connect", f_Encoder.isConnected());
    }

    public void goFState(FState pos) {
        if (pos == FState.HOME) {
            setFPos(fHome);
            fState = FState.HOME;
        
        } else if (pos == FState.LOAD_LOW) {
            setFPos(fLoadLow);
            fState = FState.LOAD_LOW;
        
        } else if (pos == FState.LOAD_CONE) {
            setFPos(fLoadCone);
            fState = FState.LOAD_CONE;
       
        } else if (pos == FState.RAMP_CONE) {
            setFPos(fRampCone);
            fState = FState.RAMP_CONE;
        
        } else if (pos == FState.RAMP_CUBE) {
            setFPos(fRampCube);
            fState = FState.RAMP_CUBE;
        
        } else if (pos == FState.SCORE_CUBE_LOW) {
            setFPos(fScoreCubeLow);
            fState = FState.SCORE_CUBE_LOW;
        
        } else if (pos == FState.SCORE_CONE_LOW) {
            setFPos(fScoreConeLow);
            fState = FState.SCORE_CONE_LOW;
        
        } else if (pos == FState.SCORE_CUBE_HIGH) {
            setFPos(fScoreCubeHigh);
            fState = FState.SCORE_CUBE_HIGH;
        
        } else if (pos == FState.SCORE_CONE_HIGH) {
            setFPos(fScoreConeHigh);
            fState = FState.SCORE_CONE_HIGH;
        
        } else if (pos == FState.FLIPPED) {
            setFPos(fFlipped);
            fState = FState.FLIPPED;
        }
    }

    public void goSState(SState pos) {
        if (pos == SState.STOP) {
            setSPos(sStop);
        } else if (pos == SState.IN) {
            setSPos(sIn);
        } else if (pos == SState.OUT) {
            setSPos(sOut);
        }
    }

    public void disableMotors() {
        flipEnable = false;
        spinEnable = false;
    }

    public void enableMotors() {
        flipEnable = true;
        spinEnable = true;
    }
}

//Max: -172 Degrees
//Min: 54630.000000 Degreees


