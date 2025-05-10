package pedroPathing.opmodes.AutoSubsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Rarm extends Subsystem {
    // BOILERPLATE
    public static final Rarm INSTANCE = new Rarm();
    private Rarm() { }

    // USER CODE
    public Servo servo;

    public String name = "Rarm";

    public Command grabSpecimen() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.91, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command setupScorespecimen() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.15, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command scorespecimen() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.15, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command endofauto() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.24, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command scorebucket() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.4, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}