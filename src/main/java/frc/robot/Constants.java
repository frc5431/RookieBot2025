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

    }

    public static class CoralRollersConstants {

        public static final boolean attached = true;
        public static final int id = 19;
        public static final double gearRatio = 4 / 1; // ASK
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

    }

    public static class AlgaeRollersConstants {

        public static final boolean attached = true;
        public static final int id = 5;
        public static final double gearRatio = 4 / 1; // ASK
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

    }

    public static class AlgaePivotConstants {

        // public enum ManipJointStates {
        // STOWED, FEED, HUMAN, L1, L2, L3, L4
        // }

        public static final boolean attached = true;
        public static final int id = 15;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(40);
        public static final Current stallLimit = Units.Amps.of(80);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = true;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kAlternateOrExternalEncoder;
        public static final double maxForwardOutput = 1;
        public static final double maxReverseOutput = -0.1;

        // public static final double s = 0.3; // 0.15 holds arm at 90 degree position,
        // when gravity's pull is strongest

        public static final double p = 2;
        public static final double i = 0.00;
        public static final double d = 1.7;
        public static final double maxIAccum = 0.005;

        public static final Angle eject = Units.Rotation.of(-0.8);
        public static final Angle stow = Units.Rotations.of(-2);

        public static final Angle groundAlgae = Units.Rotations.of(-8);
        public static final Angle processorAngle = Units.Rotations.of(-6);

        public static final Angle error = Units.Rotations.of(0.3);
        public static final Angle tightError = Units.Rotations.of(0.1);

        public enum AlgaePivotPositions {
            STOW(stow), EJECT(eject), GROUND_ALGEA(groundAlgae), PROCESSOR(processorAngle);

            public Angle position;

            AlgaePivotPositions(Angle position) {
                this.position = position;
            }

        }

    }

    public static class AutonConstants {
        // The PID values from last year
        public static final PIDConstants translationPID = new PIDConstants(1.3, 0, 0.5);
        public static final PIDConstants rotationPID = new PIDConstants(2, 0, .0);

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
