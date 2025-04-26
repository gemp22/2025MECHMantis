/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.mathfunctions.MathFunctions.AngleWrap;
import static java.lang.Math.atan2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@TeleOp(name="Teleop V1", group="Mechbot")
//@Disabled
public class MechbotTeleop_V1 extends OpMode{

    /* Declare OpMode members. */
    HardwareMechV2 robot       = new HardwareMechV2(); // use the class created to define a Pushbot's hardware

    //Gamepad Variables
    double leftStickX = 0;
    double leftStickY = 0;
    double rightStickX = 0;
    double rightTrigger = 0;
    double leftTrigger = 0;

    //Arcade drive variables
    double r = 0; //vector Magnitude
    double robotAngle = 0;  //vector angle.
    double rightX=0; // twist
    double max = 0;
    int stickPower = 10; // power to apply to stick inputs for finer control  must me an odd number

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;

    //IMU variables
    BNO055IMU imu;
    Orientation angles;

    double gyroHeadingError;
    double globalHeadingError = 0; //AutoEndHeadign;

    double IMUAngle = 0;
    double globalHeadingReSetAngle =180 ;
    double IMUreset = 0; //this is for back button to reset glabal angle mid match
    double teleOppHeading = globalHeadingReSetAngle;
    double globalAngle =0;//for Teleopp rightX heading maintainer


    //Servo variables
    double          extenderOffset  = 0;
    final double    EXTENDER_SPEED  = 0.01 ;                 // sets rate to move servo

    double          wristOffset  = 0;
    final double    WRIST_SPEED  = 0.02 ;                 // sets rate to move servo

    double          leftJunctionOffset  = 0;
    final double    LEFT_JUNCTION_SPEED  = 0.01 ;                 // sets rate to move servo

    double clawposition  = 0.05 ;
    double extenderPosition = 0;
    double wristPosition = 0;

    double leftGuidePosition = 0.00;
    double rightGuidePosition = 0.00;
    double leftGuideSetPoint = 0.0;
    double rightGuideSetPoint = 0.0;

    boolean distSensorLess;
    boolean distPreVal;
    boolean clawToggle;
    private boolean guidePreValue;
    private boolean guideToggle;

    //distance sensor variables
    double dist;
    double alpha;
    double red;
    double green;
    double blue;

    public final double SCALE_FACTOR = 255;


    final double    DriveLogPower  = 18 ;                 // sets rate to accelerate motors in grandpa's drive motor log function

    int autoTurretTicks = 0;
    int turnTablePosition=0;
    int turnTableStateMachineStartPoint = 0;


    //variables for lift

    int autoLeftLiftTicks = 0;// this gets the lift position from auto
    int autoRightLiftTicks = 0;
    int teleOpLiftSetTicks = 0;  // this is stored as abs val in the auto programs

    double liftPosition = 0;
    int liftLeftPosition=0;
    int liftRightPosition=0;

    double liftMaxTicks = 4800;

    // for lift pController input
    double minPowerLift = .001;
    double maxPowerLift = 0.99;

    // for turret pController input
    double minPowerTurret = .001;
    double maxPowerTurret = 0.99;

    //PController pController = new PController(0.01); // this is for lift and turret
    PController pControllerLiftLeft = new PController(0.003); // this is for lift left, change Kp to calibrate
    PController pControllerLiftRight = new PController(0.003); // this is for lift right, change Kp to calibrate
    PController pControllerTurret = new PController(0.003); // this is for lift


    boolean startPreValue = false; // this is for button toggle got code idea from discord
    boolean startToggle =false;

    // for back button toggles
    boolean backPreValue = false; // this is for button toggle got code idea from discord
    boolean backToggle =false;


    //rumble toggle
    boolean rumblePreVal = false;
    boolean rumbleToggle = false;

    //double intakePower = 0;

    boolean isAutoDrive = false;
    boolean isAutoLift = false;
    boolean isRecalGyro =false;
    boolean isTeleOppHeading =false;

    //Touch Pad

    boolean finger = false;
    boolean touchpadClawPreValue = false;
    boolean cycleLeft = false;
    boolean cycleLeftPreVal = false;
    boolean cyclrRight = false;
    boolean cycleRightPreVal = false;

    // Odometery
    double worldXPosition = 0.0;
    double worldYPosition = 0.0;
    double worldAngle_rad = 0.0;

    int leftEncTicks = 0;
    int rightEncTicks = 0;
    int perpEncTicks = 0;

    double l3 = 12.3; // distance between left and right encoders in inches
    double l2 = 6.15;
    double w = 2.413; //dicantace from the perp encoder to center in inches

    double c = 0.0;
    double ticksPerRev = 1440; //encoder Ticks per Revolution
    double radiusOfEncoderWheels = 1.14; //inches

    double startPointX = 0.0;
    double startPointY = 0.0;
    double startAngle = 0.0;

    double delX = 0.0;
    double delY = 0.0;
    double delAngle = 0.0; // radians

    //clocks

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();

    Gamepad.RumbleEffect customRightRumbleEffect;
    Gamepad.RumbleEffect customLeftRumbleEffect;// Use to build a custom rumble sequence.

    //Enumes for automation
    enum HomeState { //for x button
        LIFT_START_HOME,
        WRIST_FLAT,
        EXTENDER_IN,
        LIFT_MID,
        TURRET_HOME,
        LIFT_HOME,
        CLAW_HOME,
    }

    enum HomeStateRightSide { //

        LIFT_START_HOME_SIDE_RIGHT,
        EXTENDER_IN_RIGHT,
        LIFT_MID_SIDE_RIGHT,
        TURRET_HOME_SIDE_RIGHT,
        EXTENDER_HOME_RIGHT,
        LIFT_HOME_SIDE_RIGHT,

    }

    enum HomeStateLeftSide { //
        LIFT_START_HOME_SIDE_LEFT,
        EXTENDER_IN_LEFT,
        LIFT_MID_SIDE_LEFT,
        TURRET_HOME_SIDE_LEFT,
        EXTENDER_HOME_LEFT,
        LIFT_HOME_SIDE_LEFT,

    }

    enum LowState {
        LIFT_START_LOW,
        TURRET_LOW,

    }

    enum LowStateFront {
        LIFT_START_LOW_FRONT,
        TURRET_LOW_FRONT,

    }

    enum HighState { //y button
        LIFT_START_HIGH,
        TURRET_HIGH,
        LIFT_HIGH,
    }

    enum HighStateFront { //y button
        LIFT_START_HIGH_FRONT,
        TURRET_HIGH_FRONT,
        LIFT_HIGH_FRONT,
    }

    enum MidState {
        LIFT_START_MID,
        TURRET_MID,
        LIFT_MID,
    }
    enum MidSideRight {
        LIFT_START_MID_SIDE_RIGHT,
        TURRET_MID_SIDE_RIGHT,
        LIFT_MID_SIDE_RIGHT,

    }

    enum SafeDrive {
        LIFT_START_SAFE,
        TURRET_SAFE,

    }
    enum HighSideRight {
        LIFT_START_HIGH_SIDE_RIGHT,
        TURRET_HIGH_SIDE_RIGHT,
        LIFT_HIGH_SIDE_RIGHT,
    }
    enum LowSideRight {
        LIFT_START_LOW_SIDE_RIGHT,
        TURRET_LOW_SIDE_RIGHT,
        LIFT_LOW_SIDE_RIGHT,
    }

    enum HighSideLeft {
        LIFT_START_HIGH_SIDE_LEFT,
        TURRET_HIGH_SIDE_LEFT,
        LIFT_HIGH_SIDE_LEFT,
    }
    enum MidSideLeft {
        LIFT_START_MID_SIDE_LEFT,
        TURRET_MID_SIDE_LEFT,
        LIFT_MID_SIDE_LEFT,
    }
    enum LowSideLeft {
        LIFT_START_LOW_SIDE_LEFT,
        TURRET_LOW_SIDE_LEFT,
        LIFT_LOW_SIDE_LEFT,

    }


    enum LiftTurretReset { //if don't get set after auto
        LIFT_TO_SAFE,
        TURRET_TO_ZERO,
        LIFT_TO_ZERO,
    }


    HomeState homeState = HomeState.LIFT_START_HOME;
    HomeStateRightSide homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
    HomeStateLeftSide homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
    LowState lowState = LowState.LIFT_START_LOW;
    LowStateFront lowStateFront = LowStateFront.LIFT_START_LOW_FRONT;
    HighState highState = HighState.LIFT_START_HIGH;
    HighStateFront highStateFront = HighStateFront.LIFT_START_HIGH_FRONT;
    HighSideRight highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;
    HighSideLeft highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;
    MidSideLeft midSideLeft = MidSideLeft.LIFT_MID_SIDE_LEFT;
    LowSideLeft lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;
    MidState midState = MidState.LIFT_START_MID;
    MidSideRight midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
    LowSideRight lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;
    SafeDrive safeDrive = SafeDrive.LIFT_START_SAFE;
    LiftTurretReset liftTurretReset = LiftTurretReset.LIFT_TO_SAFE;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //lift  pController initalization
        pControllerLiftLeft.setInputRange(0, liftMaxTicks);
        pControllerLiftLeft.setSetPoint(0);
        pControllerLiftLeft.setOutputRange(minPowerLift, maxPowerLift);
        pControllerLiftLeft.setThresholdValue(5);

        pControllerLiftRight.setInputRange(0, liftMaxTicks);
        pControllerLiftRight.setSetPoint(0);
        pControllerLiftRight.setOutputRange(minPowerLift, maxPowerLift);
        pControllerLiftRight.setThresholdValue(5);

        //autoLeftLiftTicks = LiftStorage.getLeftEncoderTicks();// this gets the lift position from auto
       // autoRightLiftTicks = LiftStorage.getRightEncoderTicks();
        //teleOpLiftSetTicks = Math.abs(LiftStorage.getTeleOpEncoderTicks());  // this is number of ticks above teleOpp that Auto was set up in

        //turret pController initalization
        pControllerTurret.setInputRange(-2000, 2000);
        pControllerTurret.setSetPoint(0);
        pControllerTurret.setOutputRange(minPowerTurret, maxPowerTurret);
        pControllerTurret.setThresholdValue(5);

        //autoTurretTicks = TurretStorage.getTurretEncoderTicks();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu.initialize(parameters);

        robot.extender.setPosition(robot.INIT_EXTENDER);
        robot.wrist.setPosition(robot.INIT_WRIST);

        robot.claw.setPosition(clawposition);
        robot.leftGuide.setPosition(leftGuidePosition);
        robot.rightGuide.setPosition(rightGuidePosition);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        customLeftRumbleEffect = new Gamepad.RumbleEffect.Builder() //CUSTOM Rumble
                .addStep(1.0, 0.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        customRightRumbleEffect = new Gamepad.RumbleEffect.Builder() //CUSTOM Rumble
                .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(0.0, 1.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();


        //Odometery
        c = (2*Math.PI*radiusOfEncoderWheels)/ticksPerRev;


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("C", c);

        //telemetry.addData("teleOpLiftSetTicks STORAGE", LiftStorage.getTeleOpEncoderTicks());
        //telemetry.addData("autoLeftLiftTicks STORAGE", LiftStorage.getLeftEncoderTicks());
        //telemetry.addData("autoRightLiftTicks STORAGE", LiftStorage.getRightEncoderTicks());

        telemetry.addData("teleOpLiftSetTicks", teleOpLiftSetTicks);
        telemetry.addData("autoLeftLiftTicks", autoLeftLiftTicks);
        telemetry.addData("autoRightLiftTicks", autoRightLiftTicks);

        telemetry.addData("lift left position AUTO :", robot.liftLeft.getCurrentPosition()+(teleOpLiftSetTicks+autoLeftLiftTicks));
        telemetry.addData("lift right position AUTO:", robot.liftRight.getCurrentPosition()+(teleOpLiftSetTicks+autoRightLiftTicks));

        telemetry.addData("lift left position:", robot.liftLeft.getCurrentPosition());
        telemetry.addData("lift right position:", robot.liftRight.getCurrentPosition());

        telemetry.addData("AutoTurretTicks:", autoTurretTicks);
        telemetry.addData("Turret position:", robot.turnTable.getCurrentPosition()+autoTurretTicks);
        telemetry.addData("Turret position:", robot.turnTable.getCurrentPosition());

        telemetry.addData("extender:",  robot.extender.getPosition());
        telemetry.addData("wrist:",  robot.wrist.getPosition());
        telemetry.addData("claw:",  robot.claw.getPosition());

        telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double turnTable;

        //BULK READS*****************************************************************************
        //Sensor reads

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //globalAngle = angles.firstAngle - 180;
        IMUAngle = angles.firstAngle;
        globalAngle = IMUAngle - globalHeadingReSetAngle - IMUreset ;


        dist = robot.sensorDistance.getDistance(DistanceUnit.INCH); // color/dist sensor

        leftEncTicks = robot.leftRear.getCurrentPosition();
        rightEncTicks = (robot.rightFront.getCurrentPosition()*-1);
        perpEncTicks = robot.leftFront.getCurrentPosition();

        liftLeftPosition = robot.liftLeft.getCurrentPosition();
        liftRightPosition = robot.liftRight.getCurrentPosition();
        liftPosition = (liftLeftPosition + liftRightPosition) / 2;
        turnTablePosition = robot.turnTable.getCurrentPosition();

        // Game Pad Reads
        leftStickX = gamepad1.left_stick_x;  // power function on sticks does not work except for front and back bc x and y don't go all the way to 1 or other angles
        leftStickY = gamepad1.left_stick_y;
        rightStickX = Range.clip(gamepad1.right_stick_x,-.9,.9);
        rightTrigger = (gamepad1.right_trigger);
        leftTrigger = (gamepad1.left_trigger);

        //Servo Reads
        //clawposition = robot.claw.getPosition();
        wristPosition = robot.wrist.getPosition();
        extenderPosition = robot.extender.getPosition();
        leftGuidePosition = robot.leftGuide.getPosition();
        rightGuidePosition = robot.rightGuide.getPosition();

        //MOTOR CONTROL **************************************************************************

        //Finds the hypotenuse of the triangle created by the two joystick values. Used to find the absolute direction to go in.

        r = Math.hypot(leftStickX, leftStickY); // add power function here? and twist only?

        robotAngle = atan2(leftStickY * -1, leftStickX) - Math.PI / 4;  //vector angle


        // this is to toggle the reset value press back to re-set the bot heading

        if (gamepad1.back && gamepad1.back != backPreValue) { // (!=) means not equal to
            if (!backToggle) {                             // (!) Called Logical NOT Operator. Use to reverses the logical state of its operand. If a condition is true then Logical NOT operator will make false.

               IMUreset = IMUAngle; // - IMUAngle + globalHeadingReSetAngle;
               teleOppHeading = teleOppHeading - IMUreset;
            }
            backToggle = !backToggle; // ! used for inverting a boolean in Java
        }
        backPreValue = gamepad1.back;


        if (Math.abs(rightStickX) > 0.1) {
            //rightX = rightStickX;

            rightX = Math.abs(rightStickX) / rightStickX * (Math.pow(stickPower, Math.abs(rightStickX)) - 1) / (stickPower - 1);

            teleOppHeading = globalAngle; // when right stick x is near 0 exits out of if statement and locking in teleOppHeading we want

        } else {
            globalHeadingError = (globalAngle - teleOppHeading);

            while (globalHeadingError > 180) globalHeadingError -= 360;
            while (globalHeadingError <= -180) globalHeadingError += 360;
            rightX = globalHeadingError * .015;
        }

        if (!isAutoDrive) { // this turns off manual control while in auto drive so they don't conflict

            //This is for Arcade Mecahnum drive - from Game Manual 0

            v1 = r * Math.sin(robotAngle - globalAngle / 57) - rightX;
            v2 = r * Math.cos(robotAngle - globalAngle / 57) + rightX;
            v3 = r * Math.cos(robotAngle - globalAngle / 57) - rightX;
            v4 = r * Math.sin(robotAngle - globalAngle / 57) + rightX;

            if (Math.abs(v1) > 1 || Math.abs(v2) > 1 || Math.abs(v3) > 1 || Math.abs(v4) > 1) {
                // Find the largest power

                max = Math.max(Math.abs(v1), Math.abs(v2));
                max = Math.max(Math.abs(v3), max);
                max = Math.max(Math.abs(v4), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                v1 /= max;
                v2 /= max;
                v3 /= max;
                v4 /= max;
            }

            robot.rightFront.setPower(v1);
            robot.leftFront.setPower(v2);
            robot.rightRear.setPower(v3);
            robot.leftRear.setPower(v4);

        }

        //Turn Table


        if (rightTrigger > .1 && leftTrigger < .1 && turnTablePosition > -2500 && liftPosition > 725 ) {   // can only turn turret if claw is at safe elevation
            turnTable = -rightTrigger;
            turnTable = ((Math.pow(DriveLogPower, Math.abs(turnTable)) - Math.pow(DriveLogPower, 0)) / (Math.pow(DriveLogPower, 1) - Math.pow(DriveLogPower, 0))) * Math.abs(turnTable) / turnTable;
            turnTable = Math.max(turnTable,-.5); //limits max power
            robot.turnTable.setPower(turnTable);
            pControllerTurret.setSetPoint(turnTablePosition);
        }

        else if (rightTrigger > .1 && leftTrigger < .1 && turnTablePosition > -2500 && liftPosition > 2000 ) {   // can only turn turret if claw is at safe elevation
            turnTable = -rightTrigger;
            turnTable = ((Math.pow(DriveLogPower, Math.abs(turnTable)) - Math.pow(DriveLogPower, 0)) / (Math.pow(DriveLogPower, 1) - Math.pow(DriveLogPower, 0))) * Math.abs(turnTable) / turnTable;
            turnTable = Math.max(turnTable,-.5); //limits max power
            robot.turnTable.setPower(turnTable);
            pControllerTurret.setSetPoint(turnTablePosition);
        }

        else if (leftTrigger > .1 && rightTrigger < .1 && turnTablePosition < 2500 && liftPosition > 725 ) {  // can only turn turret if claw is at safe elevation
            turnTable = leftTrigger;
            turnTable = ((Math.pow(DriveLogPower, Math.abs(turnTable)) - Math.pow(DriveLogPower, 0)) / (Math.pow(DriveLogPower, 1) - Math.pow(DriveLogPower, 0))) * Math.abs(turnTable) / turnTable;
            turnTable = Math.min(turnTable,.55); //limits max power
            robot.turnTable.setPower(turnTable);
            pControllerTurret.setSetPoint(turnTablePosition);
        }

        else if (leftTrigger > .1 && rightTrigger < .1 && turnTablePosition < 2500 && liftPosition > 2000 ) {  // can only turn turret if claw is at safe elevation
            turnTable = leftTrigger;
            turnTable = ((Math.pow(DriveLogPower, Math.abs(turnTable)) - Math.pow(DriveLogPower, 0)) / (Math.pow(DriveLogPower, 1) - Math.pow(DriveLogPower, 0))) * Math.abs(turnTable) / turnTable;
            turnTable = Math.max(turnTable,.55); //limits max power
            robot.turnTable.setPower(turnTable);
            pControllerTurret.setSetPoint(turnTablePosition);
        }


        else {            //uses proportional controller to hold turret in correct spot
            if (turnTablePosition < pControllerTurret.setPoint) {  // this if else keeps turret from winding up cables
                robot.turnTable.setPower(minPowerTurret +
                        pControllerTurret.getComputedOutput(turnTablePosition));
            } else {
                robot.turnTable.setPower(minPowerTurret -
                        pControllerTurret.getComputedOutput(turnTablePosition));
            }
        }

        //lift

        if (gamepad1.dpad_up && liftPosition < liftMaxTicks) {   ///move lift up and sets controller position
            robot.liftLeft.setPower(.95);
            robot.liftRight.setPower(.95);
            pControllerLiftLeft.setSetPoint(liftLeftPosition);
            pControllerLiftRight.setSetPoint(liftRightPosition);
        }
        else if (gamepad1.dpad_down && liftPosition > 1000) {  //move lift down and sets controller position
            leftGuidePosition = 0;
            rightGuidePosition = 0;
            robot.liftLeft.setPower(-.95);
            robot.liftRight.setPower(-.95);
            pControllerLiftLeft.setSetPoint(liftLeftPosition);
            pControllerLiftRight.setSetPoint(liftRightPosition);
        }
        else if (gamepad1.dpad_down && liftPosition > 5 && Math.abs(turnTablePosition) < 20 && leftGuidePosition<.05 && rightGuidePosition<.05) {  //move lift down only if turrets facing forward and sets controller position
            leftGuidePosition = 0;
            rightGuidePosition = 0;
            robot.liftLeft.setPower(-.95);
            robot.liftRight.setPower(-.95);
            pControllerLiftLeft.setSetPoint(liftLeftPosition);
            pControllerLiftRight.setSetPoint(liftRightPosition);
        }
        else if (gamepad1.dpad_down && liftPosition > 5 &&  Math.abs(turnTablePosition)> 1970 && Math.abs(turnTablePosition)< 2030 && leftGuidePosition<.05 && rightGuidePosition<.05) {  //move lift down only if turret is facing backward and sets controller position
            leftGuidePosition = 0;
            rightGuidePosition = 0;
            robot.liftLeft.setPower(.95);
            robot.liftRight.setPower(.95);
            pControllerLiftLeft.setSetPoint(liftLeftPosition);
            pControllerLiftRight.setSetPoint(liftRightPosition);
        }
        else {                                       //uses proportional controller to hold lift in correct spot
            if (liftLeftPosition < pControllerLiftLeft.setPoint && liftRightPosition < pControllerLiftRight.setPoint) {
                robot.liftLeft.setPower(minPowerLift +
                        pControllerLiftLeft.getComputedOutput(liftLeftPosition));
                robot.liftRight.setPower(minPowerLift +
                        pControllerLiftRight.getComputedOutput(liftRightPosition));
            } else {
                robot.liftLeft.setPower(minPowerLift -
                        pControllerLiftLeft.getComputedOutput(liftLeftPosition));
                robot.liftRight.setPower(minPowerLift -
                        pControllerLiftRight.getComputedOutput(liftRightPosition));
            }
        }


        //Servos *********************************************************************************



        //extender
        if (!isAutoLift) { //check to see is auto listing is engaged
            if (gamepad1.dpad_right) {
                extenderOffset -= EXTENDER_SPEED;
            } else if (gamepad1.dpad_left) {
                extenderOffset += EXTENDER_SPEED;
            }

            extenderOffset = Range.clip(extenderOffset, 0, .2);
            robot.extender.setPosition(robot.INIT_EXTENDER + extenderOffset);


            //wrist


           if (gamepad1.right_bumper) {
                wristOffset -= WRIST_SPEED;
            } else if (gamepad1.left_bumper) {
                wristOffset += WRIST_SPEED;
            }

            wristOffset = Range.clip(wristOffset, -.3, .35);
            robot.wrist.setPosition(robot.INIT_WRIST + wristOffset);

            //Claw

            if(dist<1.7 && dist>.01 && waitTimer1.seconds() >= 1) {
            //if (dist < 1.5 && blue>1000 && green<1000 && red<1000) {
                distSensorLess = true;
            }
            else {
                distSensorLess = false;
            }

            if (distSensorLess && !distPreVal) {   //||distSensorInch<1&&!guidePreValue){

                clawToggle = !clawToggle; //switch boolean
                if (clawToggle) { // if toggle is true..
                    clawposition = .98; //close
                }

            }

            //else if (gamepad1.guide && !guidePreValue ) {  //this may work just fine if its just an if instead of else if? kept else if so they won' interfere with each other
            else if (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_1_x>-.4 && gamepad1.touchpad_finger_1_x<.4 && !touchpadClawPreValue){
                clawToggle = !clawToggle; //switch boolean
                if (clawToggle) { // if toggle is true..
                    clawposition = 0.98; // close
                }
                else {
                    clawposition = .05;//open
                    wristOffset = 0.0; // kind push cone down onto junction??
                    leftGuideSetPoint = 0;
                    rightGuideSetPoint = 0;
                    waitTimer1.reset();
                }
            }

            robot.claw.setPosition(clawposition);
            distPreVal = clawToggle;
            guidePreValue = gamepad1.guide;
            touchpadClawPreValue = gamepad1.touchpad_finger_1;


            //Junction Guides

            if (liftPosition < 3000 ) {  //if LIFT is ever below threshold tics, retract guides
                leftGuideSetPoint = 0;
                rightGuideSetPoint = 0;
            }

            if (wristPosition>.35) {  //if wrist is pointed down, retract guides
                leftGuideSetPoint = 0;
                rightGuideSetPoint = 0;
            }

            else if (liftPosition> 3500 && wristPosition<.35 &&gamepad1.right_trigger>0.15) {
                leftGuideSetPoint = 0.9;
                rightGuideSetPoint = 0;
            }

            else if (liftPosition>3500 && wristPosition<.35 && gamepad1.left_trigger>0.15) {
                leftGuideSetPoint = 0;
                rightGuideSetPoint = 0.92;
            }
             startPreValue = gamepad1.start;
             robot.leftGuide.setPosition(leftGuideSetPoint);

            startPreValue = gamepad1.start;
            robot.rightGuide.setPosition(rightGuideSetPoint);


            /// Toggle for right or left cycle
            if (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_1_x<-.8 && !cycleLeftPreVal){
                cycleLeft=true;
                if (!gamepad1.isRumbling()) {
                    gamepad1.runRumbleEffect(customLeftRumbleEffect);
                }
            }
            if (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_1_x>.8 && !cycleRightPreVal){
                cycleLeft=false;
                if (!gamepad1.isRumbling()) {
                    gamepad1.runRumbleEffect(customLeftRumbleEffect);
                }
            }

            cycleLeftPreVal = gamepad1.touchpad_finger_1;
            cycleRightPreVal = gamepad1.touchpad_finger_1;



//GAMEPAD FEED BACK ***************************************************************************

            if (runtime.seconds()>80 && !rumblePreVal) {

                gamepad1.rumble(1,1 ,2000);  //(double rumble1, double rumble2, int durationMS);
                rumblePreVal = true;
            }

            //odometery f *********************************************************************

            //for 3 wheel
            delY = c*((leftEncTicks+rightEncTicks)/2);
            //delAngle = AngleWrap(c*((rightEncTicks-leftEncTicks)/l3));
            delX = c * ((perpEncTicks-(w*((rightEncTicks-leftEncTicks)/l3))));

            worldXPosition = startPointX+(delY*Math.sin(startAngle))+(delX*Math.cos(startAngle));
            worldYPosition = startPointY+(delY*Math.cos(startAngle)) -(delX*Math.sin(startAngle));
            worldAngle_rad = startAngle+delAngle;

            //for 2 wheel
            /*
            delX = (c*leftEncTicks)+(l2*AngleWrap(IMUAngle));
            delAngle = AngleWrap(IMUAngle);
            delY = c * rightEncTicks + w*AngleWrap(IMUAngle);

            worldXPosition = startPointX+(delX*Math.cos(startAngle))-(delY*Math.sin(startAngle));
            worldYPosition = startPointY+(delX*Math.cos(startAngle)) -(delY*Math.sin(startAngle));
            worldAngle_rad = startAngle+delAngle;
            */

            if (gamepad1.guide && gamepad1.guide != guidePreValue) { // (!=) means not equal to
                if (!guideToggle) {                             // (!) Called Logical NOT Operator. Use to reverses the logical state of its operand. If a condition is true then Logical NOT operator will make false.

                    //write code for go to point
                }
                guideToggle = !guideToggle; // ! used for inverting a boolean in Java
            }
            guidePreValue = gamepad1.guide;

            // AUTOMATION ************************************************************************
            //home switch

            switch (homeState) {
                case LIFT_START_HOME:
                    // Waiting for some input
                    if (gamepad1.right_stick_button && wristOffset<.1) { // Stays in home State until x is pressed wrist ofset - is up
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the right low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        extenderOffset = 0.0;
                        leftGuideSetPoint = 0.0;
                        rightGuideSetPoint = 0.0;
                        turnTableStateMachineStartPoint = turnTablePosition; //captures turret position when button is pressed
                        waitTimer2.reset();
                        if (clawposition<=.5){
                            clawposition = 0.5; // closes claw a bit if open so it doesn't get snagged
                        }


                        homeState = HomeState.EXTENDER_IN;

                    }
                    else if (gamepad1.right_stick_button){
                        wristOffset = 0; //
                        extenderOffset = 0.08;
                        leftGuideSetPoint = 0.0;
                        rightGuideSetPoint = 0.0;
                        turnTableStateMachineStartPoint = turnTablePosition; //captures turret position when button is pressed
                        //clawposition = 0.95; // need this? may want to take cone to home?
                        waitTimer2.reset();

                        if (clawposition<=.5){
                            clawposition = 0.5; // closes claw a bit if open so it doesn't get snagged
                        }
                        homeState = HomeState.WRIST_FLAT; // goes to wrist flat state to wait for wrist to flatten out
                    }

                    break;

                case WRIST_FLAT:

                    if (waitTimer2.seconds()>.2){  // waits for wrist to flatten out before proceeding
                        homeState = HomeState.EXTENDER_IN;
                    }

                    break;

                case EXTENDER_IN:
                    if(waitTimer2.seconds()>.1) { // waits for the extender to get into position
                        extenderOffset = 0.0; // this pulls extender back in if we had to get the wrist flat
                        if (turnTablePosition >= 800 && liftPosition > 1300) {  // move turret a bit before lowering so we don't hit junction inless already close to 0
                            pControllerTurret.setSetPoint(turnTableStateMachineStartPoint - 300);
                            homeState = HomeState.LIFT_MID;
                        } else if (turnTablePosition <= -800 && liftPosition > 1300) {
                            pControllerTurret.setSetPoint(turnTableStateMachineStartPoint + 300);
                            homeState = HomeState.LIFT_MID;
                        } else if (Math.abs(turnTablePosition) > 30 && liftPosition <= 1300) {
                            pControllerLiftLeft.setSetPoint(1000);  // so the turret and lift arent in a safe orientation.. raise or lower lift to safe elevation
                            pControllerLiftRight.setSetPoint(1000);
                            homeState = HomeState.TURRET_HOME;
                        } else {
                            pControllerTurret.setSetPoint(0);
                            homeState = HomeState.LIFT_MID;
                        }
                    }
                    break;

                case LIFT_MID:
                    if (Math.abs(turnTablePosition) < 30){// && wristOffset < 0.01) { // if turret near home position skip to home state and wrist is near flat
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerTurret.setSetPoint(0);
                        homeState = HomeState.LIFT_HOME; // this sends to lift home state, skipping turret home state
                    }
                    else if(turnTablePosition >=500 && turnTablePosition < turnTableStateMachineStartPoint - 300 && liftPosition>950) { //wait until turntable has turned some before going down
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerLiftLeft.setSetPoint(1000);  // so the turret isn't in a safe orientation.. raise or lower lift to safe elevation
                        pControllerLiftRight.setSetPoint(1000);

                        homeState = HomeState.TURRET_HOME;
                    }

                    else if(turnTablePosition<-500 && turnTablePosition > turnTableStateMachineStartPoint + 300 && liftPosition>950) { //wait until turntable has turned some before going down
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerLiftLeft.setSetPoint(1000);  // so the turret isn't in a safe orientation.. raise or lower lift to safe elevation
                        pControllerLiftRight.setSetPoint(1000);

                        homeState = HomeState.TURRET_HOME;
                    }
                    else {
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerLiftLeft.setSetPoint(1000);  // so the turret isn't in a safe orientation.. raise or lower lift to safe elevation
                        pControllerLiftRight.setSetPoint(1000);
                        homeState = HomeState.TURRET_HOME;
                    }

                    break;


                case TURRET_HOME:
                    if (liftPosition > 950){// && wristPostion < 0.01) { // stays in this state until lift  gets to get to a target height then switches its to Lift home
                        wristOffset = 0.0;
                        //robot.turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        pControllerTurret.setSetPoint(0);

                        homeState = HomeState.LIFT_HOME;
                    }
                    break;

                case LIFT_HOME:
                    if (Math.abs(turnTablePosition) < 20 && wristOffset < 0.01) { //waits until turret is within 15 ticks before lowering

                        pControllerLiftLeft.setSetPoint(125);
                        pControllerLiftRight.setSetPoint(125);
                        wristOffset = 0.0;

                        //isAutoLift = !isAutoLift; //reverses is auto lift back to false so teleopp servos stuff works again
                        homeState = HomeState.CLAW_HOME;
                    }

                    break;

                case CLAW_HOME:
                    if(liftPosition<350){
                        if (clawposition<=.5){
                            clawposition = .05; //open claw
                        }

                        homeState = HomeState.LIFT_START_HOME;
                    }
                    break;
            }

            // STATEMACHINE FOR CYCLING ON RIGHT SIDE OF GROUND JUNCTION
            switch (homeStateRightSide) {
                case LIFT_START_HOME_SIDE_RIGHT:
                    // Waiting for some input
                    if (gamepad1.a && !cycleLeft) { // Stays in home State until x is pressed
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;


                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the RIGHT low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        if(liftPosition>1300) {
                            extenderOffset = 0.0;
                            leftGuideSetPoint = 0.0;
                            rightGuideSetPoint = 0.0;
                            waitTimer2.reset();


                            homeStateRightSide = HomeStateRightSide.EXTENDER_IN_RIGHT;
                        }
                        if(liftPosition<=1300) {
                            //extenderOffset = 0.0;
                            leftGuideSetPoint = 0.0;
                            rightGuideSetPoint = 0.0;
                            waitTimer2.reset();


                            homeStateRightSide = HomeStateRightSide.EXTENDER_IN_RIGHT;
                        }


                    }
                    break;

                case EXTENDER_IN_RIGHT:
                    if (waitTimer2.seconds()>.1 ) { // && liftPosition<1000 took this out 2/15

                        turnTableStateMachineStartPoint = turnTablePosition; //captures turret position when button is pressed

                        clawposition = 0.5; // need this? may want to take cone to home?
                        leftGuideSetPoint = 0.0;
                        rightGuideSetPoint = 0.0;
                        wristOffset = 0;
                        pControllerLiftLeft.setSetPoint(1500);  // so the turret and lift arent in a safe orientation.. raise or lower lift to safe elevation
                        pControllerLiftRight.setSetPoint(1500);
                        homeStateRightSide = HomeStateRightSide.LIFT_MID_SIDE_RIGHT;

                    }

                    break;

                case LIFT_MID_SIDE_RIGHT:

                    if (liftPosition>1300){// && wristOffset < 0.01) { // if turret near home position skip to home state and wrist is near flat
                        extenderOffset = 0.0;
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerTurret.setSetPoint(-300);
                        homeStateRightSide = HomeStateRightSide.EXTENDER_HOME_RIGHT; // this sends to lift home state, skipping turret home state
                    }


                    break;

                case TURRET_HOME_SIDE_RIGHT:

                    break;

                case EXTENDER_HOME_RIGHT:
                    pControllerTurret.setSetPoint(-300);
                    if(turnTablePosition < -260 && turnTablePosition > -340 && liftPosition>1300 && liftPosition<1700) { //turnTablePosition < -260 && turnTablePosition > -340 && liftPosition>1300 && liftPosition<1700
                        waitTimer2.reset();
                        extenderOffset = .15;
                        wristOffset = 0.0;
                        clawposition = 0.05;
                        homeStateRightSide = HomeStateRightSide.LIFT_HOME_SIDE_RIGHT;
                    }

                    break;

                case LIFT_HOME_SIDE_RIGHT:
                    if (Math.abs(turnTablePosition) > 260 && Math.abs(turnTablePosition) < 340 && waitTimer2.seconds()>.5) { //waits until turret is within 15 ticks and time for extender to extend before lowering

                        pControllerLiftLeft.setSetPoint(125);
                        pControllerLiftRight.setSetPoint(125);
                        wristOffset = 0.0;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                    }
                    break;

            }
               // STATEMACHINE FOR CYCLING ON LEFT SIDE OF GROUND JUNCTION
            switch (homeStateLeftSide) {
                case LIFT_START_HOME_SIDE_LEFT:
                    // Waiting for some input

                    if (gamepad1.a && cycleLeft) { // Stays in home State until x is pressed
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the RIGHT low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        if(liftPosition>1300) {
                            extenderOffset = 0.0;
                            leftGuideSetPoint = 0.0;
                            rightGuideSetPoint = 0.0;
                            waitTimer2.reset();


                            homeStateLeftSide = HomeStateLeftSide.EXTENDER_IN_LEFT;
                        }
                        if(liftPosition<=1300) {
                            //extenderOffset = 0.0;
                            leftGuideSetPoint = 0.0;
                            rightGuideSetPoint = 0.0;
                            waitTimer2.reset();


                            homeStateLeftSide = HomeStateLeftSide.EXTENDER_IN_LEFT;
                        }
                    }
                    break;

                case EXTENDER_IN_LEFT:
                    if (waitTimer2.seconds()>.1 ) {  //&& liftPosition<1000 too this out

                        turnTableStateMachineStartPoint = turnTablePosition; //captures turret position when button is pressed

                        clawposition = 0.5; // need this? may want to take cone to home?
                        leftGuideSetPoint = 0.0;
                        rightGuideSetPoint = 0.0;
                        wristOffset = 0;
                        pControllerLiftLeft.setSetPoint(1500);  // so the turret and lift arent in a safe orientation.. raise or lower lift to safe elevation
                        pControllerLiftRight.setSetPoint(1500);
                        homeStateLeftSide = HomeStateLeftSide.LIFT_MID_SIDE_LEFT;

                    }


                    break;

                case LIFT_MID_SIDE_LEFT:

                    if (liftPosition>1300){// && wristOffset < 0.01) { // if turret near home position skip to home state and wrist is near flat
                        extenderOffset = 0.0;
                        wristOffset = 0;
                        leftGuidePosition = 0.0;
                        rightGuidePosition = 0.0;
                        pControllerTurret.setSetPoint(300);
                        homeStateLeftSide = HomeStateLeftSide.EXTENDER_HOME_LEFT; // this sends to lift home state, skipping turret home state
                    }

                    break;

                case TURRET_HOME_SIDE_LEFT:

                    break;

                case EXTENDER_HOME_LEFT:
                    pControllerTurret.setSetPoint(300);
                    if(turnTablePosition > 260 && turnTablePosition < 340 && liftPosition>1300 && liftPosition<1700) {
                        waitTimer2.reset();
                        extenderOffset = .15;
                        wristOffset = 0.0;
                        clawposition = 0.05;
                        homeStateLeftSide = HomeStateLeftSide.LIFT_HOME_SIDE_LEFT;
                    }
                    break;

                case LIFT_HOME_SIDE_LEFT:
                    if (Math.abs(turnTablePosition) > 260 && Math.abs(turnTablePosition) < 340 && waitTimer2.seconds()>.5) { //waits until turret is within 15 ticks and time for extender to extend before lowering

                        pControllerLiftLeft.setSetPoint(125);
                        pControllerLiftRight.setSetPoint(125);
                        wristOffset = 0.0;

                        //isAutoLift = !isAutoLift; //reverses is auto lift back to false so teleopp servos stuff works again
                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                    }

                    break;
            }

            //**************State Machines for cycling to junctions********************
            switch (highSideLeft) {
                case LIFT_START_HIGH_SIDE_LEFT:
                    // Waiting for some input
                    if (gamepad1.x && cycleLeft) {
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, Home Left Home  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;


                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the right low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(4950); // set lift to target height
                        pControllerLiftRight.setSetPoint(4950);

                        highSideLeft = HighSideLeft.TURRET_HIGH_SIDE_LEFT;
                    }
                    break;
                case TURRET_HIGH_SIDE_LEFT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.11;
                        pControllerTurret.setSetPoint(1700);

                        highSideLeft = HighSideLeft.LIFT_HIGH_SIDE_LEFT;
                    }


                    break;
                case LIFT_HIGH_SIDE_LEFT:

                    if (liftPosition > 4700) { // extend arm for placement
                        extenderOffset = .15;
                        rightGuideSetPoint = .8;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;
                    }
                    else if (gamepad1.x) {
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;
                    }
                    break;

            }

            switch (midSideLeft) {
                case LIFT_START_MID_SIDE_LEFT:
                    // Waiting for some input
                    if (gamepad1.b && cycleLeft) {

                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, LeftHome, high  SM back start
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the right low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(3550); // set lift to target height
                        pControllerLiftRight.setSetPoint(3550);


                        midSideLeft = MidSideLeft.TURRET_MID_SIDE_LEFT;
                    }
                    break;
                case TURRET_MID_SIDE_LEFT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.1;
                        pControllerTurret.setSetPoint(-1600);

                        midSideLeft = MidSideLeft.LIFT_MID_SIDE_LEFT;
                    }

                    break;
                case LIFT_MID_SIDE_LEFT:

                    if (liftPosition > 3200) { // extend arm for placement
                        extenderOffset = .2;
                        rightGuideSetPoint = .9;
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                    }
                    else if (gamepad1.b) {
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                    }
                    break;

            }

            switch (lowSideLeft) {
                case LIFT_START_LOW_SIDE_LEFT:
                    // Waiting for some input
                    if (gamepad1.y && cycleLeft) {
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;//send the LEFT left Home, mid, high  SM back start
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the RIGHT low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(1900); // set lift to target height
                        pControllerLiftRight.setSetPoint(1900);


                        lowSideLeft = LowSideLeft.TURRET_LOW_SIDE_LEFT;
                    }
                    break;
                case TURRET_LOW_SIDE_LEFT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.1;
                        pControllerTurret.setSetPoint(-720);

                        lowSideLeft = LowSideLeft.LIFT_LOW_SIDE_LEFT;
                    }

                    break;
                case LIFT_LOW_SIDE_LEFT:

                    if (liftPosition > 1600) { // extend arm for placement
                        extenderOffset = .02;
                        leftGuideSetPoint = .9;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;
                    }
                    else if (gamepad1.y) {
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;
                    }
                    break;

            }



            switch (highSideRight) {
                case LIFT_START_HIGH_SIDE_RIGHT:
                    // Waiting for some input
                    if (gamepad1.x && !cycleLeft) {
                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the RIGHT low, mid, Right High  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;

                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(4950); // set lift to target height
                        pControllerLiftRight.setSetPoint(4950);

                        highSideRight = HighSideRight.TURRET_HIGH_SIDE_RIGHT;
                    }
                    break;
                case TURRET_HIGH_SIDE_RIGHT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.11;
                        pControllerTurret.setSetPoint(-1700);

                        highSideRight = HighSideRight.LIFT_HIGH_SIDE_RIGHT;
                    }


                    break;
                case LIFT_HIGH_SIDE_RIGHT:

                    if (liftPosition > 4700) { // extend arm for placement
                        extenderOffset = .15;
                        leftGuideSetPoint = .9;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;
                    }
                    else if (gamepad1.x) {
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;
                    }
                    break;

            }

            switch (midSideRight) {
                case LIFT_START_MID_SIDE_RIGHT:
                    // Waiting for some input
                    if (gamepad1.b && !cycleLeft) {

                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the RIGHT low, RIGHT home, high  SM back start
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;


                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(3550); // set lift to target height
                        pControllerLiftRight.setSetPoint(3550);

                        midSideRight = MidSideRight.TURRET_MID_SIDE_RIGHT;
                    }
                    break;
                case TURRET_MID_SIDE_RIGHT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.1;
                        pControllerTurret.setSetPoint(1600);

                        midSideRight = MidSideRight.LIFT_MID_SIDE_RIGHT;
                    }


                    break;
                case LIFT_MID_SIDE_RIGHT:

                    if (liftPosition > 3200) { // extend arm for placement
                        extenderOffset = .2;
                        rightGuideSetPoint = .9;
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                    }
                    else if (gamepad1.b) {
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                    }
                    break;

            }

            switch (lowSideRight) {
                case LIFT_START_LOW_SIDE_RIGHT:
                    // Waiting for some input
                    if (gamepad1.y && !cycleLeft) {

                        //send all other SMs back to start
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;//send the RIGHT RightHome, mid, high  SM back start
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        wristOffset = 0; // will set positions of servos, adjust to get prefered position
                        extenderOffset = 0;

                        pControllerLiftLeft.setSetPoint(1900); // set lift to target height
                        pControllerLiftRight.setSetPoint(1900);

                        lowSideRight = LowSideRight.TURRET_LOW_SIDE_RIGHT;
                    }
                    break;
                case TURRET_LOW_SIDE_RIGHT:

                    if (liftPosition > 1000 ){//&& turnTablePosition < -50) { //stays in state until lift position is safe height

                        wristOffset = -0.1;
                        pControllerTurret.setSetPoint(720);

                        lowSideRight = LowSideRight.LIFT_LOW_SIDE_RIGHT;
                    }


                    break;
                case LIFT_LOW_SIDE_RIGHT:

                    if (liftPosition > 1600) { // extend arm for placement
                        extenderOffset = .02;
                        leftGuideSetPoint = .9;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;
                    }
                    else if (gamepad1.y) {
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;
                    }
                    break;

            }


            switch (safeDrive) {
                case LIFT_START_SAFE:
                    // Waiting for some input
                    if (gamepad1.left_stick_button) {

                        //send all other SMs back to start
                        homeState = HomeState.LIFT_START_HOME;

                        homeStateRightSide = HomeStateRightSide.LIFT_START_HOME_SIDE_RIGHT;
                        lowSideRight = LowSideRight.LIFT_START_LOW_SIDE_RIGHT;//send the right low, mid, high  SM back start
                        midSideRight = MidSideRight.LIFT_START_MID_SIDE_RIGHT;
                        highSideRight = HighSideRight.LIFT_START_HIGH_SIDE_RIGHT;

                        homeStateLeftSide = HomeStateLeftSide.LIFT_START_HOME_SIDE_LEFT;
                        lowSideLeft = LowSideLeft.LIFT_START_LOW_SIDE_LEFT;//send the LEFT low, mid, high  SM back start
                        midSideLeft = MidSideLeft.LIFT_START_MID_SIDE_LEFT;
                        highSideLeft = HighSideLeft.LIFT_START_HIGH_SIDE_LEFT;

                        waitTimer2.reset();
                        extenderOffset = 0;
                        leftGuidePosition = 0;
                        rightGuidePosition = 0;
                        pControllerLiftLeft.setSetPoint(1500);
                        pControllerLiftRight.setSetPoint(1500);

                        if (clawposition<.5){
                            clawposition=.5;
                        }
                        safeDrive = SafeDrive.TURRET_SAFE;

                    }
                    break;
                case TURRET_SAFE:
                    if (liftPosition > 1400 && waitTimer2.seconds()>.5) {
                        extenderOffset = 0;
                        pControllerTurret.setSetPoint(0);
                        wristOffset = 0.35;
                        safeDrive = SafeDrive.LIFT_START_SAFE;
                    }
                    break;
            }

/// Telemetery ******************************************************************************************
            telemetry.addLine()
                            .addData("left Enc Ticks", leftEncTicks)
                            .addData( "right Enc Ticks", rightEncTicks)
                            .addData("Perp Enc Ticks", perpEncTicks);

            telemetry.addLine()
                            .addData("delX","%.2f",delX)
                            .addData("delY","%.2f",delY)
                            .addData("delAngle", "%.2f",Math.toDegrees(delAngle));
            telemetry.addLine()
                            .addData( "world X","%.2f",worldXPosition)
                            .addData( "world Y","%.2f", worldYPosition)
                            .addData( "World Angle", "%.2f",Math.toDegrees(worldAngle_rad));




            //telemetry.addData("runtime",runtime.seconds());

            telemetry.addLine()
                    .addData("Distance (INCH)",
                    String.format(Locale.US, "%.02f", dist));
            /*
            .addData("Alpha", alpha)
            .addData("Red  ", red)
            .addData("Green", green)
            .addData("Blue ", blue)
            .addData("Hue", hsvValues[0]);

             */
            telemetry.addLine()
                            .addData("cycleLeft",cycleLeft);

            telemetry.addLine()

                    .addData("lift left position:", liftLeftPosition)
                    .addData("lift right position:", liftRightPosition);


            telemetry.addLine()

                    .addData("Turret pos", turnTablePosition)
                   // .addData("turn table start point", turnTableStateMachineStartPoint)
                    .addData("turret set point", pControllerTurret.setPoint)
                    .addData("Turn table Power", robot.turnTable.getPower());

            telemetry.addLine()
                    .addData("dist less than",distSensorLess)
                    //.addData("dist:","%.2f",dist)
                    .addData( "claw toggle", clawToggle)
                    .addData("touchpad",touchpadClawPreValue)

                    //.addData( "Dist preVal", distPreVal);

                    //.addData("extender", "%.2f", extenderPosition)
                    .addData("wrist", "%.2f", wristOffset);
                    //.addData("claw", "%.2f", clawposition)
                    //.addData("Dist. Sensor", "%.01f in", dist);
            /*
            telemetry.addLine()
                            .addData("start preval", startPreValue)
                            .addData("start toggle",startToggle )
                            .addData("left Junct guide pos","%.2f",leftGuidePosition)
                            .addData("right Junct guide pos","%.2f",rightGuidePosition);
*/

            telemetry.addLine()
                    .addData("extender Offset", "%.2f", extenderOffset)
                    .addData("wrist Offset", "%.2f", wristOffset);
/*
            telemetry.addLine()
                    //.addData("left Stick X","%.2f", leftStickX)
                    //.addData("left Stick Y","%.2f", leftStickY)
                    .addData("IMU Reading: ", "%.2f", IMUAngle)
                    .addData("Global angle: ", "%.2f", globalAngle)
                    .addData("Global re-set angle ERROR: ", "%.2f", globalHeadingReSetAngle)
                    .addData("IMU reset: ", "%.2f", IMUreset)
                    .addData("Stick (r): ", "%.2f", robotAngle)
                    .addData("rightX: ", "%.2f", rightX)
                    .addData("Teleopp Heading", "%.2f", teleOppHeading);
*/

            /*
            telemetry.addLine()
                    .addData("V1: ", "%.1f", v1)
                    .addData("V2: ", "%.1f", v2)
                    .addData("V3: ", "%.1f", v3)
                    .addData("V4: ", "%.1f", v4);

*/
            telemetry.addLine()
                    .addData("Cycle Left?", cycleLeft)
                    .addData("HOME State", homeState)
                    .addData("SIDE RIGHT HOME State", homeStateRightSide)
                    .addData("SIDE LEFT HOME State", homeStateLeftSide);
                    //.addData("LOW State", lowState)
                    //.addData("MID State", midState)
                    //.addData("HIGH State", highState)
                    //.addData("SIDE HIGH State", highSideRight)
                    //.addData("SAFE State", safeDrive);

            telemetry.update();
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }




}
