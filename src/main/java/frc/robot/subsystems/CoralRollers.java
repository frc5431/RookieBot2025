package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CoralRollersConstants;
import frc.robot.Constants.CoralRollersConstants.CoralRollerModes;
import frc.robot.Constants.CoralRollersConstants.CoralRollerStates;
import frc.robot.Constants.CoralRollersConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class CoralRollers extends REVMechanism {

    private L1Config config = new L1Config();
    private SparkMax motor;
    public boolean attached;
    @Getter
    private CoralRollerModes mode;
    @Getter
    @Setter
    private CoralRollerStates state;

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
        this.mode = CoralRollerModes.IDLE;
        this.state = CoralRollerStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Feeder/Rollers/Mode", getMode());
        Logger.recordOutput("Feeder/Rollers/State", getState());
        Logger.recordOutput("Feeder/Rollers/Setpoint", getMode().speed.in(RPM));
        Logger.recordOutput("Feeder/Rollers/Veloci ty", getState());
        Logger.recordOutput("Feeder/Rollers/Velocity", getMotorVelocity());
        Logger.recordOutput("Feeder/Rollers/Voltage", getMotorVoltage());
        Logger.recordOutput("Feeder/Rollers/Current", getMotorCurrent());
        Logger.recordOutput("Feeder/Rollers/Output", getMotorOutput());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("CoralRollers Mode", getMode().toString());
        SmartDashboard.putString("CoralRollers State", getState().toString());
        SmartDashboard.putNumber("CoralRollers Setpoint", getMode().speed.in(RPM));
        SmartDashboard.putNumber("CoralRollers Output", getMotorOutput());
        SmartDashboard.putNumber("CoralRollers Current", getMotorCurrent());
        SmartDashboard.putNumber("CoralRollers Voltage", getMotorVoltage());
        SmartDashboard.putNumber("CoralRollers Velocity", getMotorVelocity());

        switch (this.mode) {
            case IDLE:
                setState(CoralRollerStates.IDLE);
                break;
            case OUTTAKE:
                setState(CoralRollerStates.OUTTAKE);

        }

    }

    public Command run(double speed) {
        return new RunCommand(() -> motor.set(speed),
                this).withName("CoralRollers.run");
    }

    public void runEnum(CoralRollerModes coralRollerModes, boolean rpm) {
        this.mode = coralRollerModes;
        if (rpm) {
            setVelocity(coralRollerModes.speed);
        } else {
            setPercentOutput(coralRollerModes.output);
        }
    }

    public Command runRollersCommand(CoralRollerModes coralRollerModes) {
        return new RunCommand(() -> this.runEnum(coralRollerModes, CoralRollersConstants.useRPM), this)
                .withName("CoralRollers.runEnum");
    }

    protected void setZero() {
        resetPosition();
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
