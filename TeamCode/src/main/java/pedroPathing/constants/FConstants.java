package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;

        FollowerConstants.leftFrontMotorName = "FL_Rodo";
        FollowerConstants.leftRearMotorName = "BL_Podo";
        FollowerConstants.rightFrontMotorName = "FR_Lodo";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 10.967864;

        FollowerConstants.xMovement = 69.325;
        FollowerConstants.yMovement = 26.208075;

        FollowerConstants.forwardZeroPowerAcceleration = -45.385825;
        FollowerConstants.lateralZeroPowerAcceleration = -92.74695;

        /**
         ** STEPS FOR BASIC PID TUNING **
         *** HAVE SECONDARY TUNING OFF WHEN DOING THIS STEP ***
         * P - For power which will overshoot. You want to have very little overshoot but still have power
         * I - For overshooting. Only use this once in a lifetime if you can't get D working properly for some reason.
         * D - For de-overshooting. When you increase this you remove the overshoot but too much D value will undershoot which is bad
         ** STEPS FOR SECONDARY/ADVANCED PID TUNING **
         *** USE THE GRAPH WHEN TUNING! ***
         * P - For power which will overshoot. You want to get everything as close to the 0 value as possible on the graph
         * I - For overshooting. Only use this once in a lifetime if you can't get D working properly for some reason.
         * D - For de-overshooting. You want to get everything as close to the 0 value as possible on the graph
         **/

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.25,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.005,0,0.0001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0007;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
