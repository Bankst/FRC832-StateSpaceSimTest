package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.team832.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    // The motors driving the flywheel
    private final SpeedControllerGroup flywheelMotors = new SpeedControllerGroup(
        new PWMVictorSPX(ShooterConstants.kFlywheelMotorPort1),
        new PWMVictorSPX(ShooterConstants.kFlywheelMotorPort2)
    );

    @SuppressWarnings("FieldCanBeLocal")
    private final Encoder flywheelEncoder = new Encoder(
            ShooterConstants.kFlywheelEncoderPorts[0],
            ShooterConstants.kFlywheelEncoderPorts[1]
    );

    private final PIDController flywheelPIDController;

    private FlywheelSim flywheelSimulator;
    private EncoderSim flywheelEncoderSim;

    public ShooterSubsystem() {
        flywheelEncoder.reset();

        flywheelPIDController = new PIDController(ShooterConstants.kFlywheelVelocityP, 0, 0);

        SmartDashboard.putData("flywheel/pid", flywheelPIDController);

        if (RobotBase.isSimulation()) {
            flywheelSimulator = new FlywheelSim(
                    ShooterConstants.kFlywheelPlant,
                    ShooterConstants.kFlywheelGearbox,
                    ShooterConstants.kFlywheelGearRatio,
                    null
            );
            flywheelEncoderSim = new EncoderSim(flywheelEncoder);
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            flywheelSimulator.setInput(flywheelMotors.get() * RobotController.getBatteryVoltage());
            flywheelSimulator.update(0.020);
            flywheelEncoderSim.setRate(flywheelSimulator.getAngularVelocityRadPerSec());
            SmartDashboard.putNumber("flywheel/sim_RPM", flywheelSimulator.getAngularVelocityRPM());
            SmartDashboard.putNumber("flywheel/sim_rad_per_sec", flywheelSimulator.getAngularVelocityRadPerSec());
        }
        double flywheelVoltageSetpoint = MathUtil.clamp(flywheelPIDController.calculate(flywheelSimulator.getAngularVelocityRPM()), -12.0, 12.0);
        SmartDashboard.putNumber("flywheel/pid_error", flywheelPIDController.getVelocityError());
        SmartDashboard.putNumber("flywheel/control_effort_volts", flywheelVoltageSetpoint);
        SmartDashboard.putNumber("flywheel/motor_get", flywheelMotors.get());
        flywheelMotors.set(flywheelVoltageSetpoint / 12);
    }

    @Override
    public void simulationPeriodic() {

    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return flywheelSimulator.getCurrentDrawAmps();
        } else return 0;
    }

    public void setSetpoint(double rpm) {
        flywheelPIDController.setSetpoint(rpm);
    }
}
