package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 *
 * Concept by Howard
 *
 * First Coding by Jeffrey and Alexis
 *
 *
 *
 */

/*
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//@Autonomous(name="Time Slide Op Mode", group="Pushbot")
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "Time Slice Op Mode", group = "HardwarePushbot")
//@Disabled
public class timeSliceTeleOpMode extends LinearOpMode {
    //Encoder enc;
    //enc = new Encoder(0,1,false,Encoder.EncodingType.k4X);
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //   private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //   private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    //   private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //   private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //           (WHEEL_DIAMETER_INCHES * 3.1415);
    //   private static final double DRIVE_SPEED = 0.6;
    //   private static final double TURN_SPEED = 0.5;


    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor riser = null;

    // Define class members
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    //  static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    //  static final int CYCLE_MS = 50;     // period of each cycle
    //  static final double MAX_POS = 1.0;     // Maximum rotational position
//    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;

    static final double CLAMP_MOTION_TIME = 250;

    //double clampOffset = 0;                       // Servo mid position
    //final double CLAMP_SPEED = 0.02;                   // sets rate to move servo

    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long CONTROLLERPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    final double PROPGAIN = 0.6;
    final double INTGAIN  = 0.3;
    final double DERGAIN  = 0.1;
    final long PIDMAXDUR  = 3;

    final int RISER_DISTANCE = 500;

    int rightMotorPos;
    int lefMotorPos;
    int riserMotorPos;
    int prevRiserErr = 0;

    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    public double simplePID (double err, double duration, double prevErr)
    {
        double pidmin = -.7;
        double pidmax = 0.7;
        double propTerm = Range.clip(PROPGAIN * err, pidmin, pidmax);
        double intTerm = Range.clip(duration/PIDMAXDUR,pidmin,pidmax)*INTGAIN;
        double derTerm = Range.clip((err - prevErr),pidmin,pidmax) * DERGAIN;
        return (Range.clip(propTerm + intTerm + derTerm, pidmin,pidmax));
    }

    @Override

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        long liftDuration = 0;
        long liftOffDuration = 0;

        // variables for controller inputs.
        //float g1_leftX;
        float g1_LeftY;
        //float g1_RightX;
        float g1_RightY;
        int g1_A_Counts = 0;

        double leftClamp_Cmd = LEFTUNCLAMPED;
        double rightClamp_Cmd = RIGHTUNCLAMPED;

        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float riserCmd = 0;

        boolean g1_A;
        //boolean g1_B;

        // variables to support clamp and lift
        //long clampStart = 0;
        //long liftStart = 0;


        //double legTime = CurrentTime;
        //double lastTelemetry = CurrentTime;
        //double timeLeft = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //DcMotor leftMotor = null;
        //DcMotor rightMotor = null;

        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
        int riserZero = riser.getCurrentPosition();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
  /* ***********************************************************************************************
   *****************************                CODE          ************************************
   ************************************************************************************************/


        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                // no sensors at this time.  If we add some, change this comment.
            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.
                rightMotorPos = rightDrive.getCurrentPosition();
                lefMotorPos = leftDrive.getCurrentPosition();
                riserMotorPos = riser.getCurrentPosition();

            }
            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             *         g1_LeftX    g1_RightX
             *         g1_LeftY    g1_RightY
             *         g1_a (gamepad A)
             *         g1_b (gamepad B)
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;
                //g1_leftX = gamepad1.left_stick_x;
                g1_LeftY = gamepad1.left_stick_y;
                //g1_RightX = gamepad1.right_stick_x;
                g1_RightY = gamepad1.right_stick_y;

                // do a little debounce on gamepad1.a so we don't drop the block accidentally
                // 6 counts at 30 milliseconds will delay things by 180 ms, and that allows
                // a flaky controller or a jittery operator. Splitting out the sensor "reads"
                // from the rest of the logic let's us do this here, rather than muddling up
                // the main logic of a more abstract, more complicated piece of code.
                g1_A_Counts = Range.clip((gamepad1.a)? g1_A_Counts + 1 : g1_A_Counts - 1, 0,12);
                g1_A = (g1_A_Counts >= 6);
                //g1_B = gamepad1.b;
        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
         ***************************************************************************/



        /* ********************************************************************************/
        /* vvvvvvvvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
                if (CurrentTime - LastNav > NAVPERIOD) {
                    LastNav = CurrentTime;

                    // init drive min and max to default values.  We'll reset them to other numbers
                    // if conditions demand it.
                    float driveMax = 1;
                    float driveMin = -1;
                    float riserMax = 1;
                    float riserMin = -1;
                    double riserTarget = 0;


                    // mapping inputs to servo command
                    if (g1_A){
                        // apply a limit to motor speed while the lift operation is in process
                        driveMax = (float)0.5;
                        driveMin = (float)-0.5;

                        liftDuration += NAVPERIOD;
                        liftOffDuration = 0;

                        leftClamp_Cmd = LEFTCLAMPED;
                        rightClamp_Cmd = RIGHTCLAMPED;
                        if (liftDuration > CLAMP_MOTION_TIME) {
                            // begin lift operation after allowing CLAMP_MOTION_TIME for the servos
                            // to close.
                            // using the riser encoder and/or time, put in a PID to get it to the
                            // right position and stay there.
                            int posErr = (riserZero + RISER_DISTANCE - riserMotorPos);
                            prevRiserErr = (prevRiserErr == 0)? posErr : prevRiserErr;

                            riserTarget = simplePID(posErr, liftDuration, prevRiserErr);
                            prevRiserErr = posErr;
                            // can also put in a small left/right adjustment based on bumpers
                            // BUT: PLEASE don't read the gamepad bumpers here.  This section
                            // of code has a LOT going on already.  Read the bumpers elsewhere
                            // and use them as variables here.  Can also use x axis on right stick
                            // for offset on right servo and x axis on left stick for left servo
                            // adjustments.  Just an idea.
                        }
                    }
                    else {
                        liftDuration = (liftDuration > 0) ? liftDuration - NAVPERIOD : 0;
                        liftOffDuration += NAVPERIOD;
                        leftClamp_Cmd = LEFTUNCLAMPED;
                        rightClamp_Cmd = RIGHTUNCLAMPED;
                        if (liftOffDuration >= CLAMP_MOTION_TIME) {
                            int riserErr = (riserZero - riserMotorPos);
                            riserTarget = simplePID(riserErr,liftOffDuration,prevRiserErr);
                            prevRiserErr = riserErr;
                            // allow motor to relax
                        }
                    }
                    // mapping inputs to motor commands - cube them to desensetize them around
                    // the 0,0 point.  Switching to single stick operation ought to be pretty
                    // straightforward, if that's desired.  Using 2 sticks was simpler to
                    // code up in a hurry.
                    g1_LeftY  = g1_LeftY  * g1_LeftY  * g1_LeftY;
                    g1_RightY = g1_RightY * g1_RightY * g1_RightY;


                    // The ONLY place we set the motor power variables. Set them here, and
                    // we will never have to worry about which set is clobbering the other.
                    // I aligned them this way to make it REALLY clear what's going on.
                    // Should probably think through the logic of doing the same with the servos
                    // Ideally we'd calculate a desired motor or servo action then apply any
                    // necessary clamps or limits (or overrides) right before shipping it out.

                    // Servo commands: Clipped and Clamped.

                    // motor commands: Clipped & clamped.
                    leftDriveCmd  = Range.clip(g1_LeftY,           driveMin, driveMax);
                    rightDriveCmd = Range.clip(g1_RightY,          driveMin, driveMax);
                    riserCmd      = Range.clip((float)riserTarget, riserMin, riserMax);
                }                    // END NAVIGATION


        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
        /* ********************************************************************************/



        /*  ***********************************************************************
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
                if (CurrentTime - LastServo > SERVOPERIOD) {
                    LastServo = CurrentTime;

                    // Move both servos to new position.
                    leftClamp.setPosition(leftClamp_Cmd);
                    rightClamp.setPosition(rightClamp_Cmd);
                }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
                if (CurrentTime - LastMotor > MOTORPERIOD) {
                    LastMotor = CurrentTime;
                    // Yes, we'll set the power each time, even if it's zero.
                    // this way we don't accidentally leave it somewhere.  Just simpler this way.
                    /*  Left Drive Motor Power  */
                    leftDrive.setPower(leftDriveCmd);

                    /*  Right Drive Motor Power */
                    rightDrive.setPower(rightDriveCmd);

                    /* Lifter Motor Power   */
                    riser.setPower(riserCmd);
                }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

                if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                    LastTelemetry = CurrentTime;
                    telemetry.update();
                }
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
}