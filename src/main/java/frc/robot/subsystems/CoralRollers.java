package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CoralRollersConstants;
import frc.robot.Constants.CoralRollersConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class CoralRollers extends REVMechanism {

    private L1Config config = new L1Config();
    private SparkMax motor;
    public boolean attached;

    private static class L1Config extends Config {
        public L1Config() {
            super("L1", CoralRollersConstants.id);
            configIdleMode(CoralRollersConstants.idleMode);
            configInverted(CoralRollersConstants.isInverted);
            configEncoderPosRatio(CoralRollersConstants.gearRatio);
            configPositionWrapping(false);
            configAbsoluteEncoderInverted(false);
            configPeakOutput(CoralRollersConstants.maxForwardOutput, CoralRollersConstants.maxReverseOutput);
            configMaxIAccum(CoralRollersConstants.maxIAccum);
            configPIDGains(CoralRollersConstants.p, CoralRollersConstants.i, CoralRollersConstants.d);
            configSmartCurrentLimit(CoralRollersConstants.stallLimit, CoralRollersConstants.supplyLimit);
            configPeakOutput(CoralRollersConstants.maxForwardOutput, CoralRollersConstants.maxReverseOutput);
        }
    }

    public CoralRollers(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        config.applySparkConfig(motor);

    }

    // public Command run(double speed) {
    // return new RunCommand(() -> motor.set(speed),
    // this).withName("CoralRollers.run");
    // }
    public void run(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }

}
