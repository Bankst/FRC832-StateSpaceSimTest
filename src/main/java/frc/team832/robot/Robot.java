/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team832.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
      robotContainer = new RobotContainer();
  }

  @Override
  public void simulationPeriodic() {
    double drawCurrent = -robotContainer.robotDrive.getDrawnCurrentAmps();
    double[] currentDraw = { drawCurrent };
    double loadedVoltage = BatterySim.calculateLoadedBatteryVoltage(12.8, 0.018, currentDraw);
    SmartDashboard.putNumber("current", drawCurrent);

    double leftVelo = robotContainer.robotDrive.m_drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftVelocity);
    double rightVelo = robotContainer.robotDrive.m_drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightVelocity);
    SmartDashboard.putNumber("leftVelo", leftVelo);
    SmartDashboard.putNumber("rightVelo", rightVelo);

    RoboRioSim.setVInVoltage(loadedVoltage);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
