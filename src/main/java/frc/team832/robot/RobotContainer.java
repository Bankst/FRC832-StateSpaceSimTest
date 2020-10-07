package frc.team832.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team832.lib.driverinput.controllers.Xbox360Controller;
import frc.team832.robot.subsystems.DriveSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

import java.util.List;

public final class RobotContainer {

    public final DriveSubsystem robotDrive = new DriveSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();

    final Xbox360Controller controller = new Xbox360Controller(0);
    final Button userButton = new Button(RobotController::getUserButton);

    public RobotContainer() {
        robotDrive.setDefaultCommand(new RunCommand(() -> robotDrive.arcadeDrive(-controller.getY(GenericHID.Hand.kLeft), controller.getX(GenericHID.Hand.kRight)), robotDrive));

        userButton.whenPressed(() -> shooter.setSetpoint(4000), shooter).whenReleased(() -> shooter.setSetpoint(0), shooter);
        controller.aButton.whenPressed(() -> shooter.setSetpoint(4000), shooter).whenReleased(() -> shooter.setSetpoint(0), shooter);
    }

    public Command getAutonomousCommand() {
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        7);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(6, 6, new Rotation2d(0)),
                // Pass config
                config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics,
                robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                robotDrive::tankDriveVolts,
                robotDrive
        );

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }

}
