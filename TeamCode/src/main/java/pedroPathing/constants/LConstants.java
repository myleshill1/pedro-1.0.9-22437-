package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {

        ThreeWheelIMUConstants.forwardTicksToInches = 0.0042;
        ThreeWheelIMUConstants.strafeTicksToInches = 0.0043;
        ThreeWheelIMUConstants.turnTicksToInches = 0.002;
        ThreeWheelIMUConstants.leftY = 2.5;
        ThreeWheelIMUConstants.rightY = -2.5;
        ThreeWheelIMUConstants.strafeX = -7.25;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "FR_Lodo";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "FL_Rodo";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "BL_Podo";
        //reverse and forward encoders so they are correct
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new
                RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        /*
        Rodo plugged into port 0, under the name FL_Rodo
        Podo plugged into port 1, under the name BL_Podo
        Lodo plugged into port 2, under the name FR_Lodo

         */
    }
}




