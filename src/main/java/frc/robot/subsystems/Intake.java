package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
    private final VictorSPX m_motor;
    private final DoubleSolenoid m_actuator;
    private final String m_ID;

    public Intake(int motorCANID, int[] airChannels, String ID) {
        m_ID = ID;
        m_motor = new VictorSPX(30);
        m_actuator = new DoubleSolenoid(GlobalConstants.PCMID, PneumaticsModuleType.CTREPCM, airChannels[0],
                airChannels[1]);

        m_motor.setNeutralMode(NeutralMode.Coast);

    }

    public void run(double power) {
        m_motor.set(ControlMode.PercentOutput, power);
    }

    public void extend() {
        m_actuator.set(Value.kForward);
    }

    public void retract() {
        m_actuator.set(Value.kReverse);
    }

    public void stop() {
        m_motor.set(ControlMode.PercentOutput, 0.0);
    }

    public String getID() {
        return m_ID;
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Intake " + m_ID + " RPM", m_encoder.getVelocity());
    }

}
