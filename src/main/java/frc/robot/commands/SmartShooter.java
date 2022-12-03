package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.Utilities.MathUtils;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class SmartShooter extends CommandBase {
    private final PIDController m_rotPID 
    = new PIDController(3.25, 0.0, 0.20);
    private final Shooter m_shooter;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;
    private final boolean m_updatePose;
    private final XboxController m_driver;
    private final Timer m_timer = new Timer();

    private static LinearInterpolationTable m_timeTable = ShooterConstants.kTimeTable;
    private static LinearInterpolationTable m_hoodTable = ShooterConstants.khoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.krpmTable;

    public SmartShooter(Shooter shooter, Drivetrain drive, ShooterHood hood, boolean updatePose,
             XboxController driver) {
        m_shooter = shooter;
        m_drive = drive;
        m_hood = hood;
        m_updatePose = updatePose;
        m_driver = driver;
        m_rotPID.enableContinuousInput(0, 2*Math.PI);
        //m_rotPID.setIntegratorRange(-0.25, 0.25);
        addRequirements(shooter, hood, drive);
    }

    public SmartShooter(Shooter shooter, Drivetrain drive, ShooterHood hood, boolean updatePose) {
        m_shooter = shooter;
        m_drive = drive;
        m_hood = hood;
        m_updatePose = updatePose;
        m_driver = new XboxController(4);

        m_rotPID.enableContinuousInput(0, 2*Math.PI);
        //m_rotPID.setIntegratorRange(-0.1, 0.1);
        addRequirements(shooter, hood, drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        Limelight.enable();

        //FieldRelativeSpeed speed = m_drive.getFieldRelativeSpeed();
        //FieldRelativeAccel accel = m_drive.getFieldRelativeAccel();
    
        //m_slewX.reset(speed.vx+accel.ax*ShooterConstants.kAccelCompFactor);
        //m_slewY.reset(speed.vy+accel.ay*ShooterConstants.kAccelCompFactor);
    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();

        //SmartDashboard.putNumber("Current Time", currentTime);

        //SmartDashboard.putBoolean("Shooter Running", true);

        FieldRelativeSpeed robotVel = m_drive.getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

        Translation2d target = GoalConstants.kGoalLocation;

        Translation2d robotToGoal = target.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;

        //SmartDashboard.putNumber("Calculated (in)", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = m_timeTable.getOutput(dist);

        //SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor);

            //SmartDashboard.putNumber("Goal X", virtualGoalX);
            //SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_drive.getPose().getTranslation());

            double newShotTime = m_timeTable.getOutput(toTestGoal.getDistance(new Translation2d()) * 39.37);

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }
            
            if(i == 4){
                movingGoalLocation = testGoalLocation;
                //SmartDashboard.putNumber("NewShotTime", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }

        double newDist = movingGoalLocation.minus(m_drive.getPose().getTranslation()).getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("NewDist", newDist);

        m_shooter.run(m_rpmTable.getOutput(newDist));
        m_hood.run(m_hoodTable.getOutput(newDist));


    Translation2d robotToMovingGoal = movingGoalLocation.minus(m_drive.getPose().getTranslation());


    double targetAngle = Math.atan2(robotToMovingGoal.getY(),robotToMovingGoal.getX());
    double currentAngle = MathUtils.toUnitCircAngle(m_drive.getGyro().getRadians());

    SmartDashboard.putNumber("Angular ERROR", currentAngle-targetAngle);

    double pidOutput = m_rotPID.calculate(currentAngle,targetAngle);

    if(Math.abs(currentAngle-targetAngle)<=0.15){
        m_drive.setReadytoShoot();
    }

    if(pidOutput > DriveConstants.kMaxAngularSpeed){
        pidOutput = DriveConstants.kMaxAngularSpeed;
    }
    else if(pidOutput < -DriveConstants.kMaxAngularSpeed){
        pidOutput = -DriveConstants.kMaxAngularSpeed;
    }

    SmartDashboard.putNumber("SMart ROT", pidOutput);

    double adjTranslation = ((DriveConstants.kMaxAngularSpeed-Math.abs(pidOutput))/DriveConstants.kMaxAngularSpeed)*0.30+0.05;

    //SmartDashboard.putNumber("Target Angle", targetAngle);
    //SmartDashboard.putNumber("Current Angle", currentAngle);
    //SmartDashboard.putNumber("PID Output", pidOutput);

    m_drive.drive(
    -(inputTransform(m_driver.getLeftY())
        * DriveConstants.kMaxSpeedMetersPerSecond*adjTranslation),
      -(inputTransform(m_driver.getLeftX())
        * DriveConstants.kMaxSpeedMetersPerSecond*adjTranslation),
        (pidOutput),
    true,
    false);

        if (currentTime > 0.250 && Limelight.valid() && Limelight.getDistance() >= 95.0) {
            double dL = Limelight.getDistance() * 0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tL = -1.0 * Limelight.tx();

            Pose2d pose = calcPoseFromVision(dL, tR, tL, GoalConstants.kGoalLocation);

            if (m_updatePose) {
                m_drive.setPose(pose);
            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean("Shooter Running", false);
        m_shooter.stop();
        m_hood.stop();
        m_timer.stop();
        m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driver.setRumble(RumbleType.kRightRumble, 0.0);
        Limelight.disable();
    }

    private double inputTransform(double input){
        //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
      return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.8, 0.2);
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tL, Translation2d goal) {
        double tG = tR + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }

}
