package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class Constants {
    public static class ClimberConstants {

        public enum ClimberStates {
            STOWED, CLIMB, ALIGN,
        }

        public static final boolean attached = false;
        public static final int id = 16;
        public static final double gearRatio = 135 / 1; // ASK
        public static final Current supplyLimit = Units.Amps.of(40); // ASK
        public static final Current stallLimit = Units.Amps.of(50); // ASK
        public static final double stallCurrent = 37; // ASK
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;
        public static final double p = 1; // TUNE
        public static final double i = 0.01; // TUNE
        public static final double d = 0.3; // TUNE
        public static final double maxIAccum = 0.2; // TUNE

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double stow = 0; // TODO
        public static final double in = 0.75;
        public static final double out = -0.75;
        public static final double error = 0;

        public enum ClimberPositions {
            STOW(stow), IN(in), OUT(out);

            public double speed;

            ClimberPositions(double speed) {
                this.speed = speed;
            }
        }
    }

    public static class CoralRollersConstants {

        public enum CoralRollerStates {
            IDLE, STUCK, OUTTAKE,
        }

        public static final boolean attached = true;
        public static final int id = 19;
        public static final double gearRatio = 4 / 1; // ASK
        public static final Current supplyLimit = Units.Amps.of(40); // ASK
        public static final Current stallLimit = Units.Amps.of(50); // ASK
        public static final double stallCurrent = 37; // ASK
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;
        public static final boolean useRPM = false;

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0.00065;
        public static final double i = 0.0008;
        public static final double d = 0.00003;
        public static final double maxIAccum = 2 * i;

        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum CoralRollerModes {
            IDLE(idleSpeed, 0.0), OUTTAKE(outtakeSpeed, -0.7);

            public AngularVelocity speed;
            public double output;

            CoralRollerModes(AngularVelocity speed, double output) {
                this.speed = speed;
                this.output = output;
            }
        }

    }

    public static class AlgaeRollersConstants {

        public enum AlgaeRollerStates {
            IDLE, INTAKING, FEEDING, OUTTAKING, STUCK,
        }

        public static final boolean attached = true;
        public static final int id = 5;
        public static final double gearRatio = 4 / 1; 
        public static final Current supplyLimit = Units.Amps.of(40);
        public static final Current stallLimit = Units.Amps.of(50); 
        public static final double stallCurrent = 35; // ASK
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -1;
        public static final double p = 1; // TUNE
        public static final double i = 0.01; // TUNE
        public static final double d = 0.3; // TUNE
        public static final double maxIAccum = 0.2; // TUNE
        public static final boolean useRPM = false;

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(3000);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum AlgaeRollerModes {
            IDLE(idleSpeed, 0.0), INTAKE(intakeSpeed, 0.5), FEED(feedSpeed, 0.2), OUTTAKE(outtakeSpeed, -0.5);

            public AngularVelocity speed;
            public double output;

            AlgaeRollerModes(AngularVelocity speed, double output) {
                this.speed = speed;
                this.output = output;
            }

        }
    }

    public static class AlgaePivotConstants {

        public enum AlgaePivotStates {
            STOWED, FEED, PROCESSOR
        }

        public static final boolean attached = true;
        public static final int id = 15;
        public static final double gearRatio = 36 / 1;
        public static final Current supplyLimit = Units.Amps.of(60);
        public static final Current stallLimit = Units.Amps.of(80);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kAlternateOrExternalEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -0.3;

        public static final double s = 0.05; // 0.15 holds arm at 90 degree position,
        // // when gravity's pull is strongest

        public static final double p = 0.05;
        public static final double i = 0.02;
        public static final double d = 0;
        public static final double maxIAccum = 0.005;

        public static final Angle stowAngle = Units.Rotations.of(-11.904754638671875);
        public static final Angle feedAngle = Units.Rotations.of(-64);
        public static final Angle processorAngle = Units.Rotations.of(-50);

        public static final Angle error = Units.Rotations.of(0.3);
        public static final Angle tightError = Units.Rotations.of(0.1);

        public enum AlgaePivotPositions {
            STOW(stowAngle), FEED(feedAngle), PROCESSOR(processorAngle);

            public Angle position;

            AlgaePivotPositions(Angle position) {
                this.position = position;
            }

        }

    }

    public static class AutonConstants {
        // The PID values from last year
        public static final PIDConstants translationPID = new PIDConstants(1.3, 0, 0.5);
        public static final PIDConstants rotationPID = new PIDConstants(0, 0, .0);

        // TODO: Playing with these. Need to clean up
        public static final double PATH_THETA_kP = 5;
        public static final double PATH_THETA_kI = 0.01;
        public static final double PATH_THETA_kD = 0.0;

        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI,
                2 / Math.PI);
        public static final double THETA_kP = 6;
        public static final double THETA_kI = 0.02;
        public static final double THETA_kD = 0.0;

        public static final double D_kP = 2;
        public static final double D_kI = 0.02;
        public static final double D_kD = 0;

        public static final double X_TRANSLATION_TOLERANCE = 0.05;
        public static final double Y_TRANSLATION_TOLERANCE = 0.05;

        public static final double THETA_TOLERANCE = 0.05; // Radians
    }

}
