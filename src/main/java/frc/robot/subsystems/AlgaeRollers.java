package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlgaeRollersConstants;
import frc.robot.Constants.AlgaeRollersConstants.AlgaeRollerModes;
import frc.robot.Constants.AlgaeRollersConstants.AlgaeRollerStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class AlgaeRollers extends REVMechanism {

    private AlgaeRollersConfig config = new AlgaeRollersConfig();
    private SparkMax motor;
    public boolean attached;

    @Getter
    private AlgaeRollerModes mode;
    @Getter
    @Setter
    private AlgaeRollerStates state;

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
        this.mode = AlgaeRollerModes.IDLE;
        this.state = AlgaeRollerStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("AlgaeRollers/Rollers/Mode", getMode());
        Logger.recordOutput("AlgaeRollers/Rollers/State", getState());
        Logger.recordOutput("AlgaeRollers/Rollers/Setpoint", getMode().speed.in(RPM));
        Logger.recordOutput("AlgaeRollers/Rollers/Velocity", getState());
        Logger.recordOutput("AlgaeRollers/Rollers/Velocity", getMotorVelocity());
        Logger.recordOutput("AlgaeRollers/Rollers/Voltage", getMotorVoltage());
        Logger.recordOutput("AlgaeRollers/Rollers/Current", getMotorCurrent());
        Logger.recordOutput("AlgaeRollers/Rollers/Output", getMotorOutput());

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("AlgaeRoller Mode", getMode().toString());
        SmartDashboard.putString("AlgaeRoller State", getState().toString());
        SmartDashboard.putNumber("AlgaeRoller Setpoint", getMode().speed.in(RPM));
        SmartDashboard.putNumber("AlgaeRoller Output", getMotorOutput());
        SmartDashboard.putNumber("AlgaeRoller Current", getMotorCurrent());
        SmartDashboard.putNumber("AlgaeRoller Voltage", getMotorVoltage());
        SmartDashboard.putNumber("AlgaeRoller Velocity", getMotorVelocity());

        switch (this.mode) {
            case IDLE:
                setState(AlgaeRollerStates.IDLE);
                break;
            case INTAKE:
                setState(AlgaeRollerStates.INTAKING);
                break;
            case OUTTAKE:
                setState(AlgaeRollerStates.OUTTAKING);
                break;
            case FEED:
                setState(AlgaeRollerStates.FEEDING);
                break;
        }

    }

    public Command run(double speed) {
        return new RunCommand(() -> motor.set(speed),
                this).withName("AlgaeRollers.run");
    }

    public void runEnum(AlgaeRollerModes AlgaeRollerModes, boolean rpm) {
        this.mode = AlgaeRollerModes;
        if (rpm) {
            setVelocity(AlgaeRollerModes.speed);
        } else {
            setPercentOutput(AlgaeRollerModes.output);
        }
    }

    public Command runRollersCommand(AlgaeRollerModes AlgaeRollerModes) {
        return new RunCommand(() -> this.runEnum(AlgaeRollerModes, AlgaeRollersConstants.useRPM), this)
                .withName("AlgaeRollers.runEnum");
    }

    public Command RunDefaultRollerCommand(boolean hasAlgae) {
        return new RunCommand(() -> {
            if (hasAlgae) {
                this.run(0.1);
            } else {
                this.stop();
            }
        }, this);
    }

    public void stop() {
        motor.set(0);
    }

    public void setZero() {
        resetPosition();
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }

    public boolean hasAlgae() {
        // return motor.getForwardLimitSwitch().isPressed() ||
        return motor.getOutputCurrent() >= (AlgaeRollersConstants.stallCurrent - 5);
    }

    public boolean isStalling(double tolerance) {
        return Math.abs(motor.getOutputCurrent() - AlgaeRollersConstants.stallLimit.baseUnitMagnitude()) <= tolerance;
    }
}
