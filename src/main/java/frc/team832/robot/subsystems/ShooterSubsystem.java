package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // The motors driving the flywheel
    private final SpeedControllerGroup flywheelMotors = new SpeedControllerGroup(
        new PWMVictorSPX(ShooterConstants.kFlywheelMotorPort1),
        new PWMVictorSPX(ShooterConstants.kFlywheelMotorPort2)
    );

    private final SpeedController feederMotor = new PWMVictorSPX(ShooterConstants.kFeederMotorPort);

    @SuppressWarnings("FieldCanBeLocal")
    private final Encoder flywheelEncoder = new Encoder(
            ShooterConstants.kFlywheelEncoderPorts[0],
            ShooterConstants.kFlywheelEncoderPorts[1]
    );

    @SuppressWarnings("FieldCanBeLocal")
    private final Encoder feederEncoder = new Encoder(
            ShooterConstants.kFeederEncoderPorts[0],
            ShooterConstants.kFeederEncoderPorts[1]
    );

    private final PIDController flywheelPIDController;
    private final PIDController feederPIDController;

    private FlywheelSim flywheelSimulator;
    private FlywheelSim feederSimulator;

    private EncoderSim flywheelEncoderSim;
    private EncoderSim feederEncoderSim;

    public ShooterSubsystem() {
        flywheelEncoder.reset();
        feederEncoder.reset();

        flywheelPIDController = new PIDController(ShooterConstants.kFlywheelVelocityP, 0, 0, Constants.kRobotMainLoopPeriod);
        flywheelPIDController.setTolerance(10, 50);

        feederPIDController = new PIDController(ShooterConstants.kFeederVelocityP, 0, 0, Constants.kRobotMainLoopPeriod);
        feederPIDController.setTolerance(50, 250);

        SmartDashboard.putData("flywheel/pid", flywheelPIDController);
        SmartDashboard.putData("feeder/pid", feederPIDController);

        if (RobotBase.isSimulation()) {
            flywheelSimulator = new FlywheelSim(
                    ShooterConstants.kFlywheelPlant_Char,
                    ShooterConstants.kFlywheelGearbox,
                    ShooterConstants.kFlywheelGearRatio,
                    null
            );

            feederSimulator = new FlywheelSim(
                    ShooterConstants.kFeederPlant_Char,
                    ShooterConstants.kFeederGearbox,
                    ShooterConstants.kFeederGearRatio,
                    null
            );

            flywheelEncoderSim = new EncoderSim(flywheelEncoder);
            feederEncoderSim = new EncoderSim(feederEncoder);
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            flywheelSimulator.setInput(flywheelMotors.get() * RobotController.getBatteryVoltage());
            flywheelSimulator.update(Constants.kRobotMainLoopPeriod);
            flywheelEncoderSim.setRate(flywheelSimulator.getAngularVelocityRadPerSec());

            feederSimulator.setInput(feederMotor.get() * RobotController.getBatteryVoltage());
            feederSimulator.update(Constants.kRobotMainLoopPeriod);
            feederEncoderSim.setRate(flywheelSimulator.getAngularVelocityRadPerSec());

            SmartDashboard.putNumber("flywheel/sim_RPM", flywheelSimulator.getAngularVelocityRPM());
            SmartDashboard.putNumber("flywheel/sim_rad_per_sec", flywheelSimulator.getAngularVelocityRadPerSec());
            SmartDashboard.putNumber("feeder/sim_RPM", feederSimulator.getAngularVelocityRPM());
            SmartDashboard.putNumber("feeder/sim_rad_per_sec", feederSimulator.getAngularVelocityRadPerSec());
        }

        double flywheelFFEffort = ShooterConstants.kFlywheelFF.calculate(flywheelPIDController.getSetpoint()) * ShooterConstants.kFlywheelGearRatio;
        double feederFFEffort = ShooterConstants.kFeederFF.calculate(feederPIDController.getSetpoint()) * ShooterConstants.kFeederGearRatio;

        double flywheelPIDEffort = MathUtil.clamp(flywheelPIDController.calculate(flywheelSimulator.getAngularVelocityRPM()), -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
        double feederPIDEffort = MathUtil.clamp(feederPIDController.calculate(feederSimulator.getAngularVelocityRPM()), -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

        double flywheelEffort = flywheelFFEffort + flywheelPIDEffort;
        double feederEffort = feederFFEffort + feederPIDEffort;

        SmartDashboard.putNumber("flywheel/pid_error", flywheelPIDController.getPositionError());
        SmartDashboard.putNumber("flywheel/ff_effort", flywheelFFEffort);
        SmartDashboard.putNumber("flywheel/pid_effort", flywheelPIDEffort);
        SmartDashboard.putNumber("flywheel/motor_get", flywheelMotors.get());
        SmartDashboard.putBoolean("flywheel/at_setpoint", flywheelPIDController.atSetpoint());
        SmartDashboard.putNumber("flywheel/total_effort", flywheelEffort);

        SmartDashboard.putNumber("feeder/pid_error", feederPIDController.getPositionError());
        SmartDashboard.putNumber("feeder/ff_effort", feederFFEffort);
        SmartDashboard.putNumber("feeder/pid_effort", feederPIDEffort);
        SmartDashboard.putNumber("feeder/motor_get", feederMotor.get());
        SmartDashboard.putBoolean("feeder/at_setpoint", feederPIDController.atSetpoint());
        SmartDashboard.putNumber("feeder/total_effort", feederEffort);

        flywheelMotors.set(flywheelEffort / RobotController.getBatteryVoltage());
        feederMotor.set(feederEffort / RobotController.getBatteryVoltage());
    }


    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return flywheelSimulator.getCurrentDrawAmps() + feederSimulator.getCurrentDrawAmps();
        } else return 0;
    }

    public void setFlywheelRPMSetpoint(double rpm) {
        flywheelPIDController.setSetpoint(rpm);
    }

    public void setFeederRPMSetpoint(double rpm) {
        feederPIDController.setSetpoint(rpm);
    }
}
