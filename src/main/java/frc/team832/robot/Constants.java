package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public final class Constants {
    public static final double kRobotMainLoopPeriod = 0.020;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.676;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.115;
        public static final double kvVoltSecondsPerMeter = 2.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.165;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 2.0;
        public static final double kaVoltSecondsSquaredPerRadian = .1;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearing = 88d/8d;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.1;
    }

    public static final class ShooterConstants {
        public static final int kFlywheelMotorPort1 = 4;
        public static final int kFlywheelMotorPort2 = 5;
        public static final int kFeederMotorPort = 6;

        public static final int[] kFlywheelEncoderPorts = new int[] {4, 5};
        public static final int[] kFeederEncoderPorts = new int[] {6, 7};

        // Volts to break static friction
        public static final double kFlywheelKs = 0.0437;
        public static final double kFeederKs = 0.02;

        // Volts per (rotation per second)
        private static final double kFlywheelKvRotPerSec = 0.00217;
        private static final double kFeederKvRotPerSec = 0.001;

        // Volts per (radian per second)
        public static final double kFlywheelKv = 2 * Math.PI * (kFlywheelKvRotPerSec);
        public static final double kFeederKv = 2 * Math.PI * (kFeederKvRotPerSec);

        // Volts per (rotation per second squared
        public static final double kFlywheelKaRotPerSecSq = 0.00103;
        public static final double kFeederKaRotPerSecSq = 0.000515;

        // Volts per (radian per second squared)
        public static final double kFlywheelKa = 2 * Math.PI * (kFlywheelKaRotPerSecSq);
        public static final double kFeederKa = 2 * Math.PI * (kFeederKaRotPerSecSq);

        public static final DCMotor kFlywheelGearbox = DCMotor.getNEO(2);
        public static final DCMotor kFeederGearbox = DCMotor.getNEO(1);

        // Gear ratio as output over input
        public static final double kFlywheelGearRatio = 26.0/50.0;
        public static final double kFeederGearRatio = 1.0/1.0;

        // Moment of Inertia (kg meters squared)
        public static final double kFlywheelMoI = 0.00179;
        public static final double kFeederMoI = 0.000895;

        // Linear system based on characterization data
        public static final LinearSystem<N1, N1, N1> kFlywheelPlant_Char = LinearSystemId.identifyVelocitySystem(
                kFlywheelKv, kFlywheelKa);
        public static final LinearSystem<N1, N1, N1> kFeederPlant_Char = LinearSystemId.identifyVelocitySystem(
                kFeederKv, kFeederKa);

        // Linear system based on MoI data (from CAD)
        public static final LinearSystem<N1, N1, N1> kFlywheelPlant_MoI = LinearSystemId.createFlywheelSystem(
                kFlywheelGearbox, kFlywheelMoI, kFlywheelGearRatio);
        public static final LinearSystem<N1, N1, N1> kFeederPlant_MoI = LinearSystemId.createFlywheelSystem(
                kFeederGearbox, kFeederMoI, kFeederGearRatio);

        public static double kFlywheelVelocityP = 0.0504;
        public static double kFeederVelocityP = 0.0252;

        public static SimpleMotorFeedforward kFlywheelFF = new SimpleMotorFeedforward(
                kFlywheelKs, kFlywheelKvRotPerSec, kFlywheelKaRotPerSecSq);
        public static SimpleMotorFeedforward kFeederFF = new SimpleMotorFeedforward(
                kFeederKs, kFeederKvRotPerSec, kFeederKaRotPerSecSq);
    }

    public static final class OIConstants {
        public static final int kDriverLeftStickPort = 0;
        public static final int kDriverRightStickPort = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
