package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Climber extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(ClimberConstants.kMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final ProfiledPIDController m_PID = new ProfiledPIDController(0.25, 0.0, 0.0, new Constraints(125, 200));
    private double m_pose = 1.0;

    public Climber(){
        m_motor.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_encoder.setPosition(0.0);

        m_motor.burnFlash();

    }

        // Change length of arm
        public void setDesiredPose(double pose) {
            m_PID.reset(getPose());
            m_pose = pose;
        }
    
        public void setPower(double power) {
            m_motor.set(power);
        }
    
        public void setPoseRef(double pose) {
            m_encoder.setPosition(pose);
        }
    
        public void run() {
            if (m_pose < ClimberConstants.kMinPose) {
                m_pose = ClimberConstants.kMinPose;
            } else if (m_pose > ClimberConstants.kMaxPose) {
                m_pose = ClimberConstants.kMaxPose;
            }
            m_motor.set(m_PID.calculate(m_encoder.getPosition(), m_pose));
        }
    
        @Override
        public void periodic() {
            double pose = m_encoder.getPosition();
    
            SmartDashboard.putNumber("Climber Pose", pose);
            SmartDashboard.putNumber("Climber Current", getCurrent());
    
        }
    
        public double getPose() {
            return m_encoder.getPosition();
        }
    
        public void stop() {
            m_motor.stopMotor();
        }
    
        public double getCurrent() {
            return m_motor.getOutputCurrent();
        }
    
        public boolean atSetpoint() {
            return Math.abs(m_pose - getPose()) < 1.0;
        }
    
}
