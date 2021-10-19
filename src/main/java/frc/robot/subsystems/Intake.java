package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

//import org.graalvm.compiler.lir.phases.PostAllocationOptimizationPhase.PostAllocationOptimizationContext;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.RobotMap;

/*
 * This is the Intake subsystem where anything related to the intake is found
 * 
 * Author: Harrison Lewis
 */
public class Intake extends SubsystemBase {

    /* Call intakeMotor defined in RobotMap */
    CANSparkMax intakeMotor = RobotMap.intakeMotor;
    
    /* Call intakeSolenoid defined in RobotMap */
    CANSparkMax intakeLiftMotor = RobotMap.intakeLiftMotor;
    CANEncoder intakeLiftEncoder = RobotMap.intakeLiftEncoder;

    private double intakeLiftTargetAngle;
    private double lastPos;
    private double lastTime;

    /*
     * Make this class public
     */
    boolean state = true; // false = down
    public boolean currentState = true;
    boolean countering = false;
    boolean moving = false;
    public Intake() {

        // Set up the conversion factor so the encoder tells us angle instead of motor revolutions
        intakeLiftEncoder.setPositionConversionFactor(Config.intakeLiftEncoderToAngleRatio);

        // Initialize the starting angle
        // Assumes robot code starts when lift is fully retracted;
        // it would be better to have a limit switch at the up/down position to set this
        intakeLiftEncoder.setPosition(Config.intakeLiftUpAngle);
        this.intakeLiftTargetAngle = Config.intakeLiftUpAngle;
        lastPos = Config.intakeLiftUpAngle;
        lastTime = System.currentTimeMillis();

    }

    //double downEncoderPos;
    //boolean isDownEncoderSet = false;
    double upEncoderPos = 0.0;
    //boolean isUpEncoderSet = false;
    int failCount = 0;
    public void periodicIntake() {
        
        intakeLiftController();

    }

    private void oldLiftController () {
        //boolean a = RobotBase.isEnabled();
        double RPM = Math.abs(intakeLiftEncoder.getVelocity());
        double Position = intakeLiftEncoder.getPosition();
        //System.out.println("CHECK: " + moving);
        if ( intakeLiftMotor.get() != 0.0) {
            if (RPM < 10 ) {
                failCount++;
                System.out.println("FAIL INC");
                if (failCount > 8 || countering && failCount > 8) {
                    moving = false;
                    countering = false;
                    intakeLiftMotor.stopMotor();
                    System.out.println("FAIL MAX!");
                    //System.out.println("STOP");
                    currentState = state;
                    if (state == true) {
                        upEncoderPos = Position;
                    }
                    failCount = 0;
                }
            } else {
                System.out.println("MOVE IS NOW TRUE");
                moving = true;
                failCount --;
                if (failCount < 0) {
                    failCount = 0;
                }
            }
        }
        if (state == true) {
            double motorDelta = Math.abs(Position-upEncoderPos);
            if (currentState != true || (currentState == true && motorDelta > 0.22)) {
                
                if (currentState == true && motorDelta > 0.22) {
                    intakeLiftMotor.set(0.5);    
                    System.out.println("COUNTER : " + Math.abs(Position-upEncoderPos));
                    countering = true;
                } else {
                    intakeLiftMotor.set(Config.intakeLiftSpeed); // raise
                    System.out.println("NORMAL RAISE");
                }
            }
            /*if (!isDownEncoderSet && moving == false) {
                downEncoderPos = Position;
                isDownEncoderSet = true;
                System.out.println("Down encoder set!");
            } */
            //

        } else {
            if (currentState != false ) {
                intakeLiftMotor.set(-Config.intakeDownSpeed); //owering
                System.out.println("LOWERing");
            }
            /*if (!isUpEncoderSet && moving == false) {
                upEncoderPos = Position;
                isUpEncoderSet = true;
                System.out.println("Up encoder set!");
            }*/
            //
        }
    }

    private void intakeLiftController () {

        // 1. Proportional + Derivative controller
        // 2. Feedforward controller

        // To create the feedforward controller
        // 1. Find the encoder pos where the intake lift is straight up
        // 2. Find some data points of lift angle and motor voltage needed to hold it still
        // 3. Find a sine function that fits those data points well

        // TODO Move constants to config
        double liftAngleDeg = getLiftAngle();      
        double liftAngleRad = Math.toRadians(liftAngleDeg);  

        // Gravity feedforward
        double kG = -0.25;
        double ffGravity = kG * Math.sin(liftAngleRad);

        // Proportional controller
        double error = this.intakeLiftTargetAngle - liftAngleDeg;
        double kP = 0.005;
        double pv = kP * error;
        double maxPV = 0.2;
        if (pv > maxPV) pv = maxPV;
        if (pv < -maxPV) pv = -maxPV;

        // Derivative controller
        // TODO this is super bad
        /*
        double dAngle = liftAngleDeg - lastPos;
        double dTime = (lastTime - System.currentTimeMillis()) / 1000;
        double speed = dAngle / dTime;
        double kD = 0.0015;
        double dv = kD * speed;
        dv = boundNumber(dv, 0.25);
        */

        lastPos = liftAngleDeg;
        lastTime = System.currentTimeMillis();

        double sum = ffGravity + pv;

        intakeLiftMotor.set(sum);
        SmartDashboard.putNumber("Lift Motor", ffGravity);

    }

    // TODO move this somewhere else
    private double boundNumber (double num, double limit) {
        if (Math.abs(num) > Math.abs(limit)) {
            return Math.signum(num) * limit;
        } else {
            return num;
        }
    }

    /*
     * Lower intake
     */
    public void lower() {
        //intakeSolenoid.set(false);
        //intakeLiftMotor.set(-Config.intakeDownSpeed);
        System.out.println("LOWER");
        state = false;
    }

    /*
     * Raise intake
     */
    public void raise() {
        //intakeSolenoid.set(true);
        //intakeLiftMotor.set(Config.intakeLiftSpeed);
        state = true;
    }

    /*
     * Intakes the ball
     */
    public void intake() {
        intakeMotor.set(Config.intakeSpeed);
    }

    /*
     * Stops the intake
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    public void stopLift() {
        intakeLiftMotor.stopMotor();
    }

    /*
     * Spits the ball out of the robot
     */
    public void moveOut() {
        intakeMotor.set(Config.intakeSpeed * -1);
    }

    /**
     * Get the position of the intake lift
     */
    public double getLiftAngle () {
        return intakeLiftEncoder.getPosition();
    }

}
