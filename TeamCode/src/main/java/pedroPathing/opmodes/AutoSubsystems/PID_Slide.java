package pedroPathing.opmodes.AutoSubsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config


@TeleOp
public class PID_Slide extends OpMode {



        private PIDFController SlideController;

        public static double sp = 0, si = 0, sd = 0;

        public static double sf = 0;

        public static int slidesTarget = 0;


        public DcMotorEx Rslides;
        private DcMotorEx Lslides;




        @Override
        public void init () {

            SlideController = new PIDFController(sp, si, sd, sf);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            Rslides = hardwareMap.get(DcMotorEx.class, "RSlides");
            Lslides = hardwareMap.get(DcMotorEx.class, "LSlides");

            Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Lslides.setDirection(DcMotorSimple.Direction.REVERSE);

            Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            Lslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }




        @Override
        public void loop () {

            SlideController.setPIDF(sp, si, sd, sf);

            int LslidePos = Lslides.getCurrentPosition();

            int RslidePos = Rslides.getCurrentPosition();


            double pid = SlideController.calculate(LslidePos, slidesTarget);


            double Power = pid;


            Rslides.setPower(Power);
            Lslides.setPower(Power);


            telemetry.addData("RslidePos: ", RslidePos);
            telemetry.addData("LslidePos: ", LslidePos);
            telemetry.addData("Target: ", slidesTarget);
            telemetry.addData("Power: " , Power);


            telemetry.update();

        }




}


