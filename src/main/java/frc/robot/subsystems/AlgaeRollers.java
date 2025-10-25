package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlgaeRollersConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class AlgaeRollers extends REVMechanism {

    private AlgaeRollersConfig config = new AlgaeRollersConfig();
    private SparkMax motor;
    public boolean attached;

    private static class AlgaeRollersConfig extends Config {
        public AlgaeRollersConfig() {
            super("AlgaeRollers", AlgaeRollersConstants.id);
            configIdleMode(AlgaeRollersConstants.idleMode);
            configInverted(AlgaeRollersConstants.isInverted);
            configEncoderPosRatio(AlgaeRollersConstants.gearRatio);
            configPositionWrapping(false);
            configAbsoluteEncoderInverted(false);
            configPeakOutput(AlgaeRollersConstants.maxForwardOutput, AlgaeRollersConstants.maxReverseOutput);
            configMaxIAccum(AlgaeRollersConstants.maxIAccum);
            configPIDGains(AlgaeRollersConstants.p, AlgaeRollersConstants.i, AlgaeRollersConstants.d);
            configSmartCurrentLimit(AlgaeRollersConstants.stallLimit, AlgaeRollersConstants.supplyLimit);
            configPeakOutput(AlgaeRollersConstants.maxForwardOutput, AlgaeRollersConstants.maxReverseOutput);
        }
    }

    public AlgaeRollers(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        config.applySparkConfig(motor);

    }
    

    // public Command run(double speed) {
    // return new RunCommand(() -> motor.set(speed),
    // this).withName("AlgaeRollers.run");
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
