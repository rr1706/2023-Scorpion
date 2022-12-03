package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase {
    private final VictorSPX m_motor;
    private final DigitalInput m_sensor;

    public Elevator(int motorCANID, int sensorPort, String ID) {
        m_sensor = new DigitalInput(sensorPort);
        m_motor = new VictorSPX(motorCANID);
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public void run(double power) {
        m_motor.set(ControlMode.PercentOutput, power);
    }

    public void stop() {
        m_motor.set(ControlMode.PercentOutput,0.0);
    }

    public boolean getSensor() {
        return !m_sensor.get();
    }

    @Override
    public void periodic() {
       // SmartDashboard.putNumber(m_ID + " Elevator RPM", m_encoder.getVelocity());
    }
}
