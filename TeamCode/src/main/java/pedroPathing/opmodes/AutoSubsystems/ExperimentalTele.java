package pedroPathing.opmodes.AutoSubsystems;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
public class ExperimentalTele extends LinearOpMode {


    private DcMotor FL, FR, BL, BR;
    private Servo Claw, Omni, Wrist, LArm, RArm;
    public PIDFController SlideController;

    public static double sp = 0.02, si = 0, sd = 0.0000002;

    public static double sf = 0;

    public static double sp_v = 0.025, si_v = 0, sd_v = 0.00000001;

    public static double sf_v = 0.00045;


    public static int slidesTarget = 0;


    private DcMotorEx Rslides, Lslides;

    //==================

    public PIDController PivotController;

    public static double p = 0.02, i = 0.01, d = 0.0003;

    public static double f = 0;



    public static int pivotTarget = 0;

    public final double ticks_in_degree = 2786.2 / 180;
    private DcMotorEx pivot;


    public enum STATE {

        NEUTRAL,
        MIDDLE,
        MIDDLEGRAB,
        SPECI,
        SAMPLE,


    }




    @Override
    public void runOpMode() throws InterruptedException {

        SlideController = new PIDFController(sp, si, sd, sf);
//        PivotController = new PIDController(p, i, d);



        FL = hardwareMap.dcMotor.get("FL_Rodo"); //
        BL = hardwareMap.dcMotor.get("BL_Podo"); //
        FR = hardwareMap.dcMotor.get("FR_Lodo"); //
        BR = hardwareMap.dcMotor.get("BR"); //

        pivot = hardwareMap.get(DcMotorEx.class, "pivot"); //

        Rslides = hardwareMap.get(DcMotorEx.class, "RSlides");
        Lslides = hardwareMap.get(DcMotorEx.class, "LSlides");

        Claw = hardwareMap.servo.get("Claw"); //
        Omni = hardwareMap.servo.get("Omni"); //
        Wrist = hardwareMap.servo.get("Wrist"); //

        RArm = hardwareMap.servo.get("Rarm"); //
        LArm = hardwareMap.servo.get("Larm"); //

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesTarget = 0;

        double rotation = 0.93;

        Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //We need to find out which slide/pivot should be reversed

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Rslides.setDirection(DcMotorSimple.Direction.REVERSE);


        ElapsedTime TIMER = new ElapsedTime();


        TIMER.startTime();
        boolean MIDDLE = false;
        boolean NEUTRAL = true;
        boolean SAMPLED = false;
        boolean GRAB = false;
        boolean SPECI = false;
        boolean GRABBDSPECI = false;
        boolean SCOREDSPECI = false;

        double openClaw = 0.26;
        double closedClaw = 0.49;
        double pivotArmWall = 0.91;
        double wristWall = 0.465;

        double pivotArmSpecimenScoring = 0.15;
        double wristSpecimenScoring = 0.45;

        double DRIVESPEED = 1;





        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        STATE state = STATE.NEUTRAL;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            Omni.setPosition(rotation);

            if (gamepad2.left_stick_x > 0.5) {

                rotation += 0.04;

            }

            if (-gamepad2.left_stick_x > 0.5) {

                rotation -= 0.04;

            }

            if (rotation != Omni.getPosition()) {

                Omni.setPosition(rotation);

            }

            if (rotation >= 1 && rotation > 0) {

                rotation = 1;

            } else if (rotation <= 0) {

                rotation = 0;

            }

            //---------omni math----------

            if(pivot.getCurrentPosition() < -1100){

                SlideController.setPIDF(sp, si, sd, sf);

                telemetry.addData("Horizontal" , 0);

            }
            else{

                SlideController.setPIDF(sp_v, si_v, sd_v, sf_v);

                telemetry.addData("Vertical" , 1);

            }


            int RslidePos = Rslides.getCurrentPosition();
            int LslidePos = Lslides.getCurrentPosition();


            double Rpid = SlideController.calculate(RslidePos, slidesTarget);
            double Lpid = SlideController.calculate(LslidePos, slidesTarget);


            double Rpower = Rpid;
            double LPower = Lpid;

            Rslides.setPower(Rpower);
            Lslides.setPower(LPower);

            //==============================

//            PivotController.setPID(p, i, d);
//
//            int pivotPos = pivot.getCurrentPosition();
//
//
//            double pid = PivotController.calculate(pivotPos, pivotTarget);
//
//            double ff = Math.cos(Math.toRadians(pivotTarget / ticks_in_degree)) * f;
//
//            double power = pid + ff;


//            pivot.setPower(power);

            //-------------slide pid stuff--------------


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;



            FL.setPower(frontLeftPower * DRIVESPEED);
            BL.setPower(backLeftPower * DRIVESPEED);
            FR.setPower(frontRightPower * DRIVESPEED);
            BR.setPower(backRightPower * DRIVESPEED);



            if(gamepad1.dpad_up){

                Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Lslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Rslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }



            if(gamepad2.dpad_up){
                Claw.setPosition(openClaw);
            }

            if(gamepad2.dpad_down){

                Claw.setPosition(closedClaw);

            }


            //Slide isbusy after zero for like 0.5s -> give the motors zero output


            switch (state) {

                case NEUTRAL:

                    if (NEUTRAL) {

                        DRIVESPEED = 1;

                        rotation = 0.93;

                        slidesTarget = 0;

                        LArm.setPosition(0.35);
                        RArm.setPosition(0.35);

                        Wrist.setPosition(0.35);
                        Claw.setPosition(closedClaw);

                    }

                    if (Lslides.getCurrentPosition() < 200 && NEUTRAL) {

                        pivot.setTargetPosition(-1550);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pivot.setPower(1);

                        NEUTRAL = false;
                    }

                    if (gamepad2.b) {
                        state = STATE.MIDDLE;
                        TIMER.reset();
                        MIDDLE = true;
                    }

                    if (gamepad2.x) {
                        state = STATE.SAMPLE;
                        TIMER.reset();
                        SAMPLED = true;
                    }

                    if (gamepad2.y) {
                        state = STATE.SPECI;
                        TIMER.reset();
                        SPECI = true;
                    }

                    break;
                case MIDDLE:

                    if(TIMER.seconds() >= 0.075 & pivot.getCurrentPosition() < -1350){

                        slidesTarget = 600;

                    }

                    if (TIMER.seconds() >= 0.3 && MIDDLE) {

                        LArm.setPosition(0.37);
                        RArm.setPosition(0.37);

                        Wrist.setPosition(0.35);
                        Claw.setPosition(closedClaw);

                        MIDDLE = false;
                        state = STATE.MIDDLEGRAB;
                    }

                    break;

                case MIDDLEGRAB:

                    if(gamepad2.right_bumper){

                        LArm.setPosition(0.27);
                        RArm.setPosition(0.27);
                        Wrist.setPosition(0.62);


                        TIMER.reset();
                        GRAB = true;

                    }

                    if(TIMER.seconds() > 0.21 && GRAB){
                        Claw.setPosition(closedClaw);

                        GRAB = false;
                    }

                    if(gamepad2.left_bumper){

                        LArm.setPosition(0.335);
                        RArm.setPosition(0.335);

                        Wrist.setPosition(0.62);
                        Claw.setPosition(openClaw);

                    }

                    if(gamepad2.a){
                        state = STATE.NEUTRAL;
                        NEUTRAL = true;
                    }


                    break;

                case SAMPLE:

                    if(TIMER.seconds() > 0.1 && SAMPLED){
                        DRIVESPEED = 0.8;
                        pivot.setTargetPosition(0);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pivot.setPower(1);


                    }

                    if (TIMER.seconds() > 0.45 && SAMPLED && pivot.getCurrentPosition() > -100 ) {

                        rotation = 0.65;

                        slidesTarget = 1350;

                    }

                    if (TIMER.seconds() > 0.65 && Lslides.getCurrentPosition() > 700 && SAMPLED){

                        LArm.setPosition(0.37);
                        RArm.setPosition(0.37);

                        Wrist.setPosition(0.2);

                        SAMPLED = false;

                    }

                    if (gamepad2.right_trigger > 0.75){

                        Claw.setPosition(openClaw);


                    }

                    if(gamepad2.dpad_left){

                        state = STATE.NEUTRAL;

                        NEUTRAL = true;

                    }


                    break;

                case SPECI:
                    if (SPECI && TIMER.seconds() >= 0.2) {

                        Claw.setPosition(openClaw);

                        slidesTarget = 0;
                    }
                    if (Lslides.getCurrentPosition() < 200 && SPECI && TIMER.seconds() >= 0.5) {

                        pivot.setTargetPosition(0);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pivot.setPower(1);

                        SPECI = false;

                    }
                    if (gamepad2.left_bumper) {
                        LArm.setPosition(pivotArmWall);
                        RArm.setPosition(pivotArmWall);
                        Wrist.setPosition(wristWall);

                        Claw.setPosition(openClaw);
                    }


                    if (gamepad2.right_bumper) {
                        Claw.setPosition(closedClaw);

                    }

                    if (gamepad2.x){

                        slidesTarget = 325;

                        TIMER.reset();
                        GRABBDSPECI = true;

                    }

                    if (TIMER.seconds() > 0.2 && GRABBDSPECI) {

                        LArm.setPosition(pivotArmSpecimenScoring);
                        RArm.setPosition(pivotArmSpecimenScoring);
                        Wrist.setPosition(wristSpecimenScoring);

                        GRABBDSPECI = false;

                    }

                    if(gamepad2.a){

                        slidesTarget = 750;

                    }

                    if(gamepad2.b){

                        Claw.setPosition(openClaw);

                        TIMER.reset();
                        SCOREDSPECI = true;

                    }

                    if(SCOREDSPECI && TIMER.seconds() > 0.2) {

                        LArm.setPosition(pivotArmWall);
                        RArm.setPosition(pivotArmWall);
                        Wrist.setPosition(wristWall);

                        Claw.setPosition(openClaw);

                        SCOREDSPECI = false;
                        TIMER.reset();
                        SPECI = true;


                    }


                    break;



            }


            telemetry.addLine("MECHANISMS:");
            telemetry.addData("RSlidePos: ", RslidePos);
            telemetry.addData("LSlidePos: ", LslidePos);
            telemetry.addData("Target: ", slidesTarget);
            telemetry.addLine();
            telemetry.addData("PivotPos", pivot.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine("SERVOS:");
            telemetry.addData("RArmPos:" , RArm.getPosition());
            telemetry.addData("LArmPos:" , LArm.getPosition());
            telemetry.addLine();
            telemetry.addData("WristPos:" , LArm.getPosition());
            telemetry.addData("OmniPos:" , rotation);
            telemetry.addLine("VOLTAGE");
            telemetry.addData("PivotV", pivot.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LSlidesV", Lslides.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RSlidesV", Rslides.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine();
            telemetry.addLine("EMERGENCY SLIDE RESET BUTTON IS D-PAD UP ON GAMEPAD1");





            telemetry.update();

        }
    }
}



