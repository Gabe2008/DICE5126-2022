package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name = "Revy2v5clean (Blocks to Java)")
public class Revy2v5clean extends LinearOpMode {

  private VuforiaCurrentGame vuforiaFreightFrenzy;
  private DcMotor ArmMotor;
  private DcMotor Right;
  private DcMotor ElbowMotor;
  private DcMotor spinner0;
  private DcMotor Left;
  private DistanceSensor RearRight_DistanceSensor;
  private DistanceSensor RearRightOuter;
  private DistanceSensor RearLeftInner_DistanceSensor;
  private DistanceSensor RearLeftOuter;
  private TouchSensor ArmParked;
  private Servo Servo1;
  private CRServo spinner1;
  private TouchSensor MagneticLimit;
  private TouchSensor elbowtouch;

  int tgtPosition;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int tgtLowerArm;
    int tgtElbow;
    double maxSpeed;
    double LowerArmPower;
    boolean parkedOnce;
    ElapsedTime runtime;
    double tgtPower;
    double tgtPower2;
    int currentElbowVelocity;
    double parkPosition;

    vuforiaFreightFrenzy = new VuforiaCurrentGame();
    ArmMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
    Right = hardwareMap.get(DcMotor.class, "Right");
    ElbowMotor = hardwareMap.get(DcMotor.class, "Elbow Motor");
    spinner0 = hardwareMap.get(DcMotor.class, "spinner0");
    Left = hardwareMap.get(DcMotor.class, "Left");
    RearRight_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RearRight");
    RearRightOuter = hardwareMap.get(DistanceSensor.class, "RearRightOuter");
    RearLeftInner_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RearLeftInner");
    RearLeftOuter = hardwareMap.get(DistanceSensor.class, "RearLeftOuter");
    ArmParked = hardwareMap.get(TouchSensor.class, "ArmParked");
    Servo1 = hardwareMap.get(Servo.class, "Servo 1");
    spinner1 = hardwareMap.get(CRServo.class, "spinner1");
    MagneticLimit = hardwareMap.get(TouchSensor.class, "Magnetic Limit");
    elbowtouch = hardwareMap.get(TouchSensor.class, "elbow touch");

    gamepad1.rumble(1000);
    tgtLowerArm = 0;
    tgtElbow = 0;
    maxSpeed = 1;
    // Put initialization blocks here.
    vuforiaFreightFrenzy.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
    LowerArmPower = -0.5;
    parkedOnce = false;
    runtime = new ElapsedTime();
    ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    Right.setDirection(DcMotorSimple.Direction.REVERSE);
    ElbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    spinner0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    tgtPosition = 0;
    // above Makes sure motors are going the right direction and sets variables to their starting values
    if (opModeIsActive()) {
      // Sets up motor settings
      // Arm motors use encoder to hold position
      ElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      ElbowMotor.setTargetPosition(ElbowMotor.getCurrentPosition());
      while (opModeIsActive()) {
        // Display sensor readout
        telemetry.addData("Target Power", tgtPower);
        tgtPower = getSpeed() * -maxSpeed;
        telemetry.addData("RearRight Inner (CM)", RearRight_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("RearRight Outer (CM)", RearRightOuter.getDistance(DistanceUnit.CM));
        telemetry.addData("RearLeft Inner (CM)", RearLeftInner_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("RearLeft Outer (CM)", RearLeftOuter.getDistance(DistanceUnit.CM));
        telemetry.addData("Motor Power", Left.getPower());
        tgtPower2 = getSteer() * -(maxSpeed * 0.7);
        // Sets wheel motor power for driving
        Left.setPower(-tgtPower2 + tgtPower);
        Right.setPower(tgtPower2 + tgtPower);
        telemetry.addData("Target Power", tgtPower2);
        telemetry.addData("Motor Power", Right.getPower());
        telemetry.addData("arm position", ArmMotor.getCurrentPosition());
        telemetry.addData("elbowPosition", ElbowMotor.getCurrentPosition());
        telemetry.addData("tgtPosition", tgtPosition);
        // Above prints data
        if (getElbow() < -0.2) {
          // Moves arm down
          tgtPosition = (int) (getElbow() * 40 + tgtPosition);
          ElbowMotor.setTargetPosition(tgtPosition);
          ElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          currentElbowVelocity = 2000;
          ((DcMotorEx) ElbowMotor).setVelocity(currentElbowVelocity);
          runtime.reset();
        } else if (ArmParked.isPressed()) {
          powerDownElbowMotor();
        } else if (getElbow() > 0.2) {
          // Moves arm up
          if (ArmParked.isPressed()) {
            powerDownElbowMotor();
          } else {
            tgtPosition = (int) (getElbow() * 40 + tgtPosition);
            ElbowMotor.setTargetPosition(tgtPosition);
            ElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentElbowVelocity = 2000;
            ((DcMotorEx) ElbowMotor).setVelocity(currentElbowVelocity);
            runtime.reset();
          }
        } else if (runtime.seconds() <= 30) {
          // Countdown to check if motor is idle
          telemetry.addData("arm hold time remaining", 30 - runtime.seconds());
        } else {
          powerDownElbowMotor();
        }
        if (gamepad1.x) {
          // Lowers driving speed
          maxSpeed = 0.5;
        }
        if (gamepad1.b) {
          // Sets driving speed back to default
          maxSpeed = 1;
        }
        if (gamepad2.a) {
          // Opens claw
          Servo1.setPosition(1);
        } else {
          // Closes slaw
          Servo1.setPosition(0.57);
        }
        if (gamepad2.b) {
          // Spins duck spinner
          spinner1.setPower(1);
        } else {
          spinner1.setPower(0);
        }
        telemetry.addData("Magnetic Switch Status", MagneticLimit.isPressed());
        telemetry.addData("Elbow Button Status", elbowtouch.isPressed());
        telemetry.addData("Arm Parked Button Status", ArmParked.isPressed());
        telemetry.addData("Elbow Position", ElbowMotor.getCurrentPosition());
        telemetry.addData("Lower Arm Position", ArmMotor.getCurrentPosition());
        telemetry.addData("Park Position", parkPosition);
        // Above prints out data to screen
        if (MagneticLimit.isPressed() || ArmParked.isPressed()) {
          // Prevents arm from going to low or too high (damaging floor/ motor/ or robot)
          if (getLowerArm() < 0) {
            ArmMotor.setPower(getLowerArm() * LowerArmPower);
          } else {
            ArmMotor.setPower(0);
          }
        } else {
          if (getLowerArm() > 0) {
            ArmMotor.setPower(getLowerArm() * LowerArmPower);
          } else {
            if (elbowtouch.isPressed()) {
              ArmMotor.setPower(0);
            } else {
              ArmMotor.setPower(getLowerArm() * LowerArmPower);
            }
          }
        }
        telemetry.update();
        if (gamepad2.b) {
          // Spins duck spinner if button is pressed
          spinner0.setPower(-1);
        } else {
          spinner0.setPower(0);
        }
      }
    }

    vuforiaFreightFrenzy.close();
  }

  /**
   * Checks if forward or reverse
   */
  private double getSpeed() {
    float inSpeed;

    inSpeed = gamepad1.left_stick_y;
    if (inSpeed == 0) {
      inSpeed = -1 * gamepad1.right_trigger;
    }
    if (inSpeed == 0) {
      inSpeed = 1 * gamepad1.left_trigger;
    }
    return inSpeed;
  }

  /**
   * Gets right stick x
   */
  private double getSteer() {
    float inSteer;

    inSteer = gamepad1.right_stick_x;
    telemetry.addData("getSteer", inSteer);
    return inSteer;
  }

  /**
   * if arm is stuck or idle it powers down to avoid motor stall which can damage motor
   */
  private void powerDownElbowMotor() {
    tgtPosition = -20;
    ElbowMotor.setPower(0);
    telemetry.addData("arm hold time remaining", 0);
  }

  /**
   * Checks if the dpad is being pressed or the right stick is being
   * moved. The reason for the function is to allow more than one button
   * to control. Dpad moves both elbow and arm motors at the same time.
   */
  private double getElbow() {
    float inElbow;

    inElbow = gamepad2.right_stick_y;
    if (inElbow == 0) {
      if (gamepad2.dpad_up) {
        inElbow = -1;
      }
    }
    if (inElbow == 0) {
      if (gamepad2.dpad_down) {
        inElbow = 1;
      }
    }
    return inElbow;
  }

  /**
   * Checks if the dpad is being pressed or the left stick
   * is being moved. The reason for the function is to
   * allow more than one button to control the lower arm.
   */
  private double getLowerArm() {
    float inLowerArm;

    inLowerArm = gamepad2.left_stick_y;
    if (inLowerArm == 0) {
      if (gamepad2.dpad_up) {
        inLowerArm = 1;
      }
    }
    if (inLowerArm == 0) {
      if (gamepad2.dpad_down) {
        inLowerArm = -1;
      }
    }
    return inLowerArm;
  }
}
