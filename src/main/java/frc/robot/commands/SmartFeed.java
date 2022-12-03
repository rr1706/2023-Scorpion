package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class SmartFeed extends CommandBase {
    private final Elevator m_top;
    private final Drivetrain m_drive;
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final XboxController m_operator;
    private final Timer m_timer = new Timer();
    private boolean m_End = false;
    private double m_shotDistLimiter = 320;
    private double m_shotTime = 0.0;
    private boolean m_hasShot = false;

    public SmartFeed(Elevator top, Drivetrain drive, Shooter shooter, ShooterHood hood,
            XboxController operator) {
        m_top = top;
        m_drive = drive;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = operator;
        addRequirements(top);
    }

    public SmartFeed(Elevator top, Elevator bottom, Drivetrain drive, Shooter shooter, ShooterHood hood,
            double shotDistLimiter) {
        m_top = top;
        m_drive = drive;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = new XboxController(3);
        m_shotDistLimiter = shotDistLimiter;
        addRequirements(top, bottom);
    }

    @Override
    public void initialize() {
        m_top.stop();
        m_End = false;
        m_timer.reset();
        m_timer.start();
        m_shotTime = 0.0;
        m_hasShot = false;
    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();
        boolean isOmegaLow = Math.abs(m_drive.getFieldRelativeSpeed().omega) < Math.PI / 3.0;
        boolean isAlphaLow = true;//Math.abs(m_drive.getFieldRelativeAccel().alpha) <= 2.00;
        boolean isDriveReady = m_drive.isReadyToShoot();
        boolean isShooterReady = m_shooter.atSetpoint();
        boolean isHoodReady = m_hood.atSetpoint();

        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;
        boolean isClose = dist < m_shotDistLimiter;

        boolean[] array = { isOmegaLow, isAlphaLow, isDriveReady, isShooterReady, isHoodReady };

        //SmartDashboard.putBooleanArray("Booleans", array);

         if(!m_hasShot){
            m_top.run(0.60);
            m_hasShot = true;
            m_shotTime = currentTime;
        }
        else if(m_hasShot && (currentTime - m_shotTime) < 0.200){
            m_top.run(0.60);
        }
        else if(m_hasShot && (currentTime - m_shotTime) >= 0.300){
            m_hasShot = false;
        }
        else if(m_hasShot && (currentTime - m_shotTime) >= 0.200){
            m_top.stop();
        } 


        //SmartDashboard.putBoolean("Shooting", true);

    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_top.stop();
        //SmartDashboard.putBoolean("Shooting", false);
        m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
        m_operator.setRumble(RumbleType.kRightRumble, 0.0);
    }

    public void stop() {
        m_End = true;
    }

    @Override
    public boolean isFinished() {
        return m_End;
    }
}
