package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SmartFeed;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.*;

public class oneball extends SequentialCommandGroup {

    public oneball(Drivetrain drive, Intake intake, Shooter shooter, ShooterHood hood, Elevator top, XboxController operator) {

        final AutoFromPathPlanner oneballone = new AutoFromPathPlanner(drive, "1-ball-1", 1.5, true);

        final SmartShooter shoot = new SmartShooter(shooter, drive, hood, true);

        final SmartFeed feed = new SmartFeed(top, drive, shooter, hood, operator);

        addCommands(
            new InstantCommand(() -> drive.resetOdometry(oneballone.getInitialPose())),
            
            oneballone,
            shoot.raceWith(new WaitCommand(3.0).andThen(feed))
        );
    }

}
