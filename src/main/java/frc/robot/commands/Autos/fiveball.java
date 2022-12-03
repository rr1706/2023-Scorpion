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

public class fiveball extends SequentialCommandGroup {

    public fiveball(Drivetrain drive, Intake intake, Shooter shooter, ShooterHood hood, Elevator top, XboxController operator) {

        final AutoFromPathPlanner fiveballone = new AutoFromPathPlanner(drive, "5-ball-1", 2.5,true);
        final AutoFromPathPlanner fiveballtwo = new AutoFromPathPlanner(drive, "5-ball-2", 2.5, true);
        final AutoFromPathPlanner fiveballthree = new AutoFromPathPlanner(drive, "5-ball-3", 3.5, true);

        final SmartShooter shoot1 = new SmartShooter(shooter, drive, hood, true);
        final SmartShooter shoot2 = new SmartShooter(shooter, drive, hood, true);

        final SmartFeed feed1 = new SmartFeed(top, drive, shooter, hood, operator);
        final SmartFeed feed2 = new SmartFeed(top, drive, shooter, hood, operator);

        addCommands(
            new InstantCommand(() -> drive.resetOdometry(fiveballone.getInitialPose())),
            
            fiveballone.raceWith(new RunIntake(intake)),
            shoot1.raceWith(new WaitCommand(0.75).andThen(feed1.alongWith(new WaitCommand(0.60).andThen(new RunIntake(intake).withTimeout(0.30))).withTimeout(2.0))),
            fiveballtwo.raceWith(new IndexElevator(top)).andThen(new WaitCommand(1.0)).raceWith(new RunIntake(intake)),                    
            fiveballthree.raceWith(new IndexElevator(top)),
            shoot2.raceWith(new WaitCommand(0.75).raceWith(new RunIntake(intake)).andThen(feed2.withTimeout(5.0)))
        
        );

    }

}
