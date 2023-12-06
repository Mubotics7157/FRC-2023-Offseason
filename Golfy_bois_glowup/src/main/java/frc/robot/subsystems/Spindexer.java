package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.util.CommonConversions;

public class Spindexer extends SubsystemBase{

    public enum SpindexerState{
        OFF,
        IDLE,
        INTAKING,
        SHOOTING
    }
    
    private static Spindexer instance = new Spindexer();

    private CANSparkMax spindexerMotor = new CANSparkMax(SpindexerConstants.DEVICE_ID_SPINDEXER, MotorType.kBrushless);
    private CANSparkMax throatMotor = new CANSparkMax(SpindexerConstants.DEVICE_ID_THROAT, MotorType.kBrushless);

    private SpindexerState currentState = SpindexerState.OFF;
    public Spindexer(){
        spindexerMotor.restoreFactoryDefaults();
        throatMotor.restoreFactoryDefaults();

        spindexerMotor.setInverted(false);
        throatMotor.setInverted(false);

        spindexerMotor.setIdleMode(IdleMode.kCoast);
        throatMotor.setIdleMode(IdleMode.kBrake);

        SparkMaxPIDController controller = spindexerMotor.getPIDController();
        controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        controller.setSmartMotionMaxAccel(SpindexerConstants.MAX_ACCELERATION_RPM * SpindexerConstants.SPINDEXER_GEARING, 0);
        controller.setSmartMotionMaxVelocity(SpindexerConstants.MAX_VELOCITY_RPM * SpindexerConstants.SPINDEXER_GEARING, 0);

        controller.setP(SpindexerConstants.SPINDEXER_KP);
        controller.setD(SpindexerConstants.SPINDEXER_KD);
        controller.setFF(SpindexerConstants.SPINDEXER_KF);
    }

    public static Spindexer getInstance(){
        if(instance != null)
            return instance;
        else
            return new Spindexer();
    }

    @Override
    public void periodic() {
        
    }
}
