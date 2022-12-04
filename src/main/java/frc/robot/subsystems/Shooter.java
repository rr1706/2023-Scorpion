package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    private final PIDController m_PID = new PIDController(ShooterConstants.kPID[0], ShooterConstants.kPID[1],
            ShooterConstants.kPID[2]);
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(ShooterConstants.kStatic, ShooterConstants.kFF);

    private double m_RPM = ShooterConstants.kMaxRPM;

    private double[] m_lastPosition1 = {0,0,0,0,0,0,0};
    private double m_velocity = 0.0;
    private double[] m_lastTime = {0,0,0,0,0,0,0};
    private Timer m_timer = new Timer(); 

    public Shooter(int motorIDs[]) {

        m_motor1 = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
        m_encoder1 = m_motor1.getEncoder();
        m_encoder2 = m_motor2.getEncoder();

        m_motor1.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor2.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor1.setInverted(true);
        m_motor2.follow(m_motor1, true);

        m_encoder1.setVelocityConversionFactor(1.0);
        m_encoder2.setVelocityConversionFactor(1.0);

        m_encoder1.setPosition(0.0);

        m_motor1.burnFlash();
        m_motor2.burnFlash();

        m_PID.setTolerance(ShooterConstants.kRPMTolerance);
        //m_PID.setIntegratorRange(0, 0);

        m_PID.setIntegratorRange(-ShooterConstants.kIntRange, ShooterConstants.kIntRange);
        
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);

        m_timer.start();
    }

    public void run(double rpm) {
        if(SmartDashboard.getBoolean("Adjust Shot?", true)){
            rpm = rpm + SmartDashboard.getNumber("SetShotAdjust", 0.0);
        }
        if (rpm >= ShooterConstants.kMaxRPM) {
            rpm = ShooterConstants.kMaxRPM;
        }
        m_RPM = rpm;
        double outputPID = m_PID.calculate(getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        double output = outputPID + outputFF;

        if (output <= ShooterConstants.kMaxNegPower) {
            output = ShooterConstants.kMaxNegPower;
        }

        m_motor1.set(outputPID + outputFF);
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }

    @Override
    public void periodic() {

        double currentTime = m_timer.get();

        double position1 = m_encoder1.getPosition();
        double deltaTime = currentTime-m_lastTime[0];
        
        SmartDashboard.putNumber("Shooter time delta", deltaTime);
        SmartDashboard.putNumberArray("Time Array", m_lastTime);

        m_velocity = 60.0*((position1 - m_lastPosition1[0])/(deltaTime));


        for(int i=0;i<6;i++){
            if(i<=5){
                m_lastTime[i] = m_lastTime[i+1];
                m_lastPosition1[i] = m_lastPosition1[i+1];
            }
        }

        m_lastTime[6] = currentTime;
        m_lastPosition1[6] = position1;

        SmartDashboard.putNumber("Shooter RPM Custom", getVelocity());
        SmartDashboard.putNumber("Shooter RPM Spark", m_encoder1.getVelocity());
        SmartDashboard.putNumber("Shooter Error", m_RPM-getVelocity());

    }

    public double getVelocity(){
        return m_velocity;
    }

    public boolean atSetpoint() {
        return m_PID.atSetpoint();
    }

}
