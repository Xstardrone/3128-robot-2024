package frc.team3128.subsystems;

import common.core.controllers.PIDFFConfig;
import common.core.controllers.TrapController;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team3128.subsystems.Climber.Setpoint;
import frc.team3128.subsystems.Intake.IntakePivot;
import frc.team3128.subsystems.Intake.IntakeRollers;
import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

public class Intake2 {
    public enum Setpoints {
        extended(120),
        retracted(0);

        public double angle;
        private Setpoints(double angle) {
            this.angle = angle;
        }
    }

    public class Intake2Pivot extends PivotTemplate {

        private Intake2Pivot() {
            super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);
            initShuffleboard();
            System.out.println("Start PID and Shuffleboard");
        }

        @Override
        protected void configMotors() {
            PIVOT_MOTOR.setInverted(true);
            PIVOT_MOTOR.setUnitConversionFactor(360 * GEAR_RATIO);
            PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
            PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }

        @Override
        public void useOutput(double output, double setpoint) {
            PIVOT_MOTOR.setVolts(MathUtil.clamp(output, -12, 12));
        }

        public Command hardReset(double power) {
            return sequence(
                waitUntil(()-> intakePivot.atSetpoint()).withTimeout(1.5),
                intakePivot.runPivot(power),
                waitSeconds(0.1),
                intakePivot.runPivot(0),
                intakePivot.reset(0)
            );
        }

        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(() -> startPID(setpoint.getAsDouble()));
        }

        public SetpointTest getPivotTest() {
            return new SetpointTest(
                "testIntakePivot",
                Setpoints.extended.angle,
                SETPOINT_TEST_PLATEAU,
                SETPOINT_TEST_TIMEOUT
            );
        }
    }

    public class Intake2Rollers extends ManipulatorTemplate {
        private DigitalInput limitSwitch;

        private Intake2Rollers() {
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, RIGHT_ROLLER_MOTOR);
            limitSwitch = new DigitalInput(9);
            initShuffleboard();
        }

        @Override
        public void configMotors() {
            RIGHT_ROLLER_MOTOR.setInverted(false);
            LEFT_ROLLER_MOTOR.setInverted(true);
            RIGHT_ROLLER_MOTOR.enableVoltageCompensation(9);
            LEFT_ROLLER_MOTOR.follow(RIGHT_ROLLER_MOTOR);
            RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            RIGHT_ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            RIGHT_ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        public Command runNoRequirements(double power) {
            return new InstantCommand(() -> setPower(power));
        }

        public Command outtakeWithTimeout(double timeout) {
            return sequence(
                runNoRequirements(OUTTAKE_POWER),
                waitSeconds(timeout),
                runNoRequirements(0)
            );
        }

        public Command ampOuttake(double timeout) {
            return sequence(
                runNoRequirements(-0.3),
                waitSeconds(timeout),
                runNoRequirements(0)
            );
        }

        public Command serialize() {
            return sequence(
                runManipulator(-0.1),
                waitUntil(()-> !hasObjectPresent()),
                runManipulator(0.1),
                waitUntil(() -> hasObjectPresent()),
                runManipulator(0)
            );
        }

        @Override
        public boolean hasObjectPresent() {
            return !limitSwitch.get();
        }
    }

    private static Intake2 instance;

    public Intake2Pivot intake2pivot;
    public Intake2Rollers intake2Rollers;

    public boolean isRetracting = false;

    public static synchronized Intake2 getInstance() {
        if (instance == null) {
            instance = new Intake2();
        }
        return instance;
    }

    private Intake2() {
        intake2pivot = new Intake2Pivot();
        intake2Rollers = new Intake2Rollers();
    }
}