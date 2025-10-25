package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPositions;
import frc.robot.Constants.ClimberConstants.ClimberStates;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class Climber extends REVMechanism {

    private ClimberConfig config = new ClimberConfig();
    private SparkMax motor;
    public boolean attached;

    @Getter
    private ClimberPositions mode;
    @Getter
    @Setter
    private ClimberStates state;

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
        this.mode = ClimberPositions.STOW;
        this.state = ClimberStates.STOWED;
        config.applySparkConfig(motor);

        Logger.recordOutput("ClimberMode", getMode());
        Logger.recordOutput("Climber/State", getState());
        Logger.recordOutput("Climber/Velocity", getMotorVelocity());
        Logger.recordOutput("Climber/Voltage", getMotorVoltage());
        Logger.recordOutput("Climber/Current", getMotorCurrent());
        Logger.recordOutput("Climber/Output", getMotorOutput());
        Logger.recordOutput("Climber/Velocity", getMotorPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber Mode", this.getMode().toString());
        SmartDashboard.putNumber("Climber Position", this.getMotorPosition());

        SmartDashboard.putNumber("Climber Output", this.getMotorOutput());
        SmartDashboard.putNumber("Climber Current", this.getMotorCurrent());
        SmartDashboard.putNumber("Climber Voltage", this.getMotorVoltage());
        SmartDashboard.putNumber("Climber Position", this.getMotorPosition());

    }

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

    // public Command run(double speed) {
    // return new RunCommand(() -> motor.set(speed), this).withName("Climber.run");
    // }
    public void run(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }

    public void setZero() {
        resetPosition();
    }

    public double getMotorPosition() {
        if (attached) {
            return motor.getEncoder().getPosition();
        }

        return 0;
    }

    public void runEnum(ClimberPositions Climbermode) {
        this.mode = Climbermode;
        this.setPercentOutput(Climbermode.speed);
    }

    public Command runClimberCommand(ClimberPositions Climbermode) {
        return new StartEndCommand(() -> this.runEnum(Climbermode), () -> this.setPercentOutput(0))

                .withName("Climber.runEnum");
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }

}
