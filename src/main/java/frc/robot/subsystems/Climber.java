package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ClimberConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Climber extends REVMechanism {

    private ClimberConfig config = new ClimberConfig();
    private SparkMax motor;
    public boolean attached;

    private static class ClimberConfig extends Config {
        public ClimberConfig() {
            super("Climber", ClimberConstants.id);
            configIdleMode(ClimberConstants.idleMode);
            configInverted(ClimberConstants.isInverted);
            configEncoderPosRatio(ClimberConstants.gearRatio);
            configPositionWrapping(false);
            configAbsoluteEncoderInverted(false);
            configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
            configMaxIAccum(ClimberConstants.maxIAccum);
            configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
            configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
            configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
        }
    }

    public Climber(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        config.applySparkConfig(motor);

    }

    // public Command run(double speed) {
    // return new RunCommand(() -> motor.set(speed), this).withName("Climber.run");
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
