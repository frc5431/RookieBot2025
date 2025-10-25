package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.AlgaePivotConstants.AlgaePivotPositions;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class AlgaePivot extends REVMechanism {

    private AlgaePivotConfig config = new AlgaePivotConfig();
    private SparkMax motor;
    public boolean attached;

    private static class AlgaePivotConfig extends Config {
        public AlgaePivotConfig() {
            super("AlgaePivot", AlgaePivotConstants.id);
            configIdleMode(AlgaePivotConstants.idleMode);
            configInverted(AlgaePivotConstants.isInverted);
            configEncoderPosRatio(AlgaePivotConstants.gearRatio);
            configPositionWrapping(false);
            configAbsoluteEncoderInverted(false);
            configPeakOutput(AlgaePivotConstants.maxForwardOutput, AlgaePivotConstants.maxReverseOutput);
            configMaxIAccum(AlgaePivotConstants.maxIAccum);
            configPIDGains(AlgaePivotConstants.p, AlgaePivotConstants.i, AlgaePivotConstants.d);
            configSmartCurrentLimit(AlgaePivotConstants.stallLimit, AlgaePivotConstants.supplyLimit);
            configPeakOutput(AlgaePivotConstants.maxForwardOutput, AlgaePivotConstants.maxReverseOutput);
        }
    }

    public AlgaePivot(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        config.applySparkConfig(motor);

    }

    // public Command run(double speed) {
    // return new RunCommand(() -> motor.set(speed),
    // this).withName("AlgaePivot.run");
    // }

    public void run(double speed) {
        motor.set(speed);
    }

    public void runEnum(AlgaePivotPositions algaePivotPositions) {
        setMotorPosition(algaePivotPositions.position);
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
