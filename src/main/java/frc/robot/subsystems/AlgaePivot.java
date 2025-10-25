package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.AlgaePivotConstants.AlgaePivotPositions;
import frc.robot.Constants.AlgaePivotConstants.AlgaePivotStates;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class AlgaePivot extends REVMechanism {

    private AlgaePivotConfig config = new AlgaePivotConfig();
    private SparkMax motor;
    public boolean attached;

    @Getter
    private AlgaePivotPositions mode;
    @Getter
    @Setter
    private AlgaePivotStates state;

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
            configFeedForwardGains(AlgaePivotConstants.s, AlgaePivotConstants.p, AlgaePivotConstants.i,
                    AlgaePivotConstants.d);
            configSmartCurrentLimit(AlgaePivotConstants.stallLimit, AlgaePivotConstants.supplyLimit);
            configPeakOutput(AlgaePivotConstants.maxForwardOutput, AlgaePivotConstants.maxReverseOutput);
        }
    }

    public AlgaePivot(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        this.mode = AlgaePivotPositions.STOW;
        this.state = AlgaePivotStates.STOWED;
        config.applySparkConfig(motor);

        Logger.recordOutput("AlgaePivot/Joint/Mode", getMode());
        Logger.recordOutput("AlgaePivot/Joint/Setpoint", getMode().position.in(Rotation));
        Logger.recordOutput("AlgaePivot/Joint/State", getState());
        Logger.recordOutput("AlgaePivot/Joint/Velocity", getMotorVelocity());
        Logger.recordOutput("AlgaePivot/Joint/Voltage", getMotorVoltage());
        Logger.recordOutput("AlgaePivot/Joint/Current", getMotorCurrent());
        Logger.recordOutput("AlgaePivot/Joint/Output", getMotorOutput());
        Logger.recordOutput("AlgaePivot/Joint/Velocity", getMotorPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("AlgaePivot Mode", this.getMode().toString());
        SmartDashboard.putNumber("AlgaePivot Setpoint", getMode().position.in(Rotations));
        SmartDashboard.putBoolean("AlgaePivot Goal",
                getPositionSetpointGoal(getMode().position, AlgaePivotConstants.error));
        SmartDashboard.putNumber("AlgaePivot Output", this.getMotorOutput());
        SmartDashboard.putNumber("AlgaePivot Current", this.getMotorCurrent());
        SmartDashboard.putNumber("AlgaePivot Voltage", this.getMotorVoltage());
        SmartDashboard.putNumber("AlgaePivot Position", this.getMotorPosition());

    }

    /**
     * Checks if the motor is reaching the rotational setpoint
     * 
     * @param target
     *               the target rotation angle
     * @param error
     *               allowed error in rotations (keep SMALL)
     * @return true if the motor's angle position is within the error of the target
     *         angle position
     */
    @Override
    public boolean getPositionSetpointGoal(Angle target, Angle error) {
        if (attached) {
            if (Calc.approxEquals(motor.getEncoder().getPosition(), target.in(Rotations),
                    error.in(Rotations))) {
                return true;
            }
        }
        return false;
    }

    public void stop() {
        motor.set(0);
    }

    public void setZero() {
        motor.getEncoder().setPosition(0);
    }

    public void runEnum(AlgaePivotPositions algaePivotPositions) {
        this.mode = algaePivotPositions;
        setMotorPosition(algaePivotPositions.position);
    }

    public Command runAlgaePivotCommand(AlgaePivotPositions AlgaePivotmode) {
        return new RunCommand(() -> this.runEnum(AlgaePivotmode), this)
                .withName("AlgaePivot.runEnum").withTimeout(1);
    }

    public Command run(double speed) {
        return new RunCommand(() -> motor.set(speed),
                this).withName("AlgaeRollers.run");
    }

    public double getMotorPosition() {
        if (attached) {
            return motor.getEncoder().getPosition();
        }

        return 0;
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }
}
