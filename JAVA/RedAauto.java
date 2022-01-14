package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@Autonomous(name = "RedAauto (Blocks to Java)")
public class RedAauto extends LinearOpMode {

  private DcMotor Right;
  private Servo Servo1;
  private VuforiaCurrentGame vuforiaFreightFrenzy;
  private TfodCurrentGame tfodFreightFrenzy;
  private DistanceSensor RearRightOuter;
  private DistanceSensor RearLeftOuter;
  private DcMotor ArmMotor;
  private DcMotor ElbowMotor;
  private DcMotor Left;
  private DcMotor spinner0;
  private DistanceSensor RearRight_DistanceSensor;
  private DistanceSensor RearLeftInner_DistanceSensor;

  int level;
  int elbowTarget;
  int rightTarget;
  int armTarget;
  int leftTarget;
  Recognition recognition;
  double DRIVE_COUNTS_PER_IN;
  List leftList;
  List rightList;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // TODO: Enter the type for variable named toLevel
    UNKNOWN_TYPE toLevel;
    int nextState;
    int COUNTS_PER_REV;
    int DRIVE_GEAR_REDUCTION;
    double WHEEL_CIRCUMFERENCE_MM;
    double DRIVE_COUNTS_PER_MM;
    List<Recognition> recognitions;
    int index;

    Right = hardwareMap.get(DcMotor.class, "Right");
    Servo1 = hardwareMap.get(Servo.class, "Servo 1");
    vuforiaFreightFrenzy = new VuforiaCurrentGame();
    tfodFreightFrenzy = new TfodCurrentGame();
    RearRightOuter = hardwareMap.get(DistanceSensor.class, "RearRightOuter");
    RearLeftOuter = hardwareMap.get(DistanceSensor.class, "RearLeftOuter");
    ArmMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
    ElbowMotor = hardwareMap.get(DcMotor.class, "Elbow Motor");
    Left = hardwareMap.get(DcMotor.class, "Left");
    spinner0 = hardwareMap.get(DcMotor.class, "spinner0");
    RearRight_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RearRight");
    RearLeftInner_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RearLeftInner");

    // Created by gabe and michael from DICE5126
    // https://docs.revrobotics.com/kickoff-concepts/freight-frenzy-2021-2022/autonomous
    // Used rev robotics tutorial for precision autonomous drivng
    nextState = 0;
    COUNTS_PER_REV = 288;
    DRIVE_GEAR_REDUCTION = 1;
    WHEEL_CIRCUMFERENCE_MM = 111 * Math.PI;
    DRIVE_COUNTS_PER_MM = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    Right.setDirection(DcMotorSimple.Direction.REVERSE);
    Servo1.setPosition(1);
    // We are no longer using the vuforia code
    // Initialize Vuforia.
    vuforiaFreightFrenzy.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
    // Set min confidence threshold to 0.7
    tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodFreightFrenzy.activate();
    // Enable following block to zoom in on target.
    tfodFreightFrenzy.setZoom(1.2, 16 / 9);
    telemetry.addData("rearRightOuter (cm)", RearRightOuter.getDistance(DistanceUnit.CM));
    telemetry.addData("rearLeftOuter (cm)", RearLeftOuter.getDistance(DistanceUnit.CM));
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Wait for start command from Driver Station.
    Servo1.setPosition(0);
    telemetry.speak("Revy is Ready", null, null);
    sleep(2000);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Get a list of recognitions from TFOD.
        recognitions = tfodFreightFrenzy.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
          telemetry.addData("TFOD", "No items detected.");
        } else {
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            // Display info.
            displayInfo(index);
            // Increment index.
            index = index + 1;
          }
        }
        telemetry.addData("duckSpinnerPower", spinner0.getPower());
        telemetry.addData("rearRightOuter (cm)", RearRightOuter.getDistance(DistanceUnit.CM));
        telemetry.addData("rearLeftOuter (cm)", RearLeftOuter.getDistance(DistanceUnit.CM));
        telemetry.addData("rearRightInner (cm)", RearRight_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("rearLeftInner (cm)", RearLeftInner_DistanceSensor.getDistance(DistanceUnit.CM));
        // State machine
        if (nextState == 0) {
          // Initial state after Play pressed
          nextState = 1;
        } else if (nextState == 1) {
          level = readBarcode();
          nextState = 2;
        } else if (nextState == 2) {
          turn135CCW();
          nextState = 3;
        } else if (nextState == 3) {
          liftArm(level);
          nextState = 4;
        } else if (nextState == 4) {
          driveToHub(level);
          nextState = 5;
        } else if (nextState == 5) {
          dropCube();
          nextState = 6;
        } else if (nextState == 6) {
          backUp(toLevel);
          nextState = 7;
        } else if (nextState == 7) {
          turn45CW();
          nextState = 8;
        } else if (nextState == 8) {
          driveToWarehouse();
          nextState = 9;
        } else if (nextState == 9) {
          nextState = 10;
          telemetry.speak("Done", null, null);
        } else {
          // Done. Waiting for stop.
        }
        // Prints data to screen
        telemetry.addData("State", nextState);
        telemetry.addData("Arm CurrentPosition", ArmMotor.getCurrentPosition());
        telemetry.addData("Elbow CurrentPosition", ElbowMotor.getCurrentPosition());
        telemetry.addData("Left CurrentPosition", Left.getCurrentPosition());
        telemetry.addData("Right CurrentPosition", Right.getCurrentPosition());
        telemetry.addData("Level", level);
        telemetry.update();
      }
    }
    // Deactivate TFOD.
    tfodFreightFrenzy.deactivate();

    vuforiaFreightFrenzy.close();
    tfodFreightFrenzy.close();
  }

  /**
   * drives to warehouse while parking arm (adjusted distance for level 2)
   */
  private void driveToWarehouse() {
    if (2 == level) {
      drive2andlower(0.7, 64, 64);
    } else {
      drive2andlower(0.8, 64, 64);
    }
  }

  /**
   * lowers arm
   */
  private void park() {
    if (opModeIsActive()) {
      elbowTarget = 0;
      armTarget = 0;
      ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ArmMotor.setTargetPosition(armTarget);
      ElbowMotor.setTargetPosition(elbowTarget);
      ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmMotor.setPower(1);
      ElbowMotor.setPower(0.1);
      while (opModeIsActive() && (ElbowMotor.isBusy() || ArmMotor.isBusy())) {
      }
      Left.setPower(0);
      Right.setPower(0);
    }
    drive(1, 0, 0);
  }

  /**
   * drives into warehouse while lowering arm in to parked position
   *
   * combined drive block and move arm block
   */
  private void drive2andlower(double power, int leftInches, int rightInches) {
    if (opModeIsActive()) {
      elbowTarget = 0;
      armTarget = 0;
      ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ArmMotor.setTargetPosition(armTarget);
      ElbowMotor.setTargetPosition(elbowTarget);
      ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmMotor.setPower(1);
      ElbowMotor.setPower(0.05);
      rightTarget = (int) (Right.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN);
      leftTarget = (int) (Left.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ((DcMotorEx) Left).setTargetPositionTolerance(20);
      ((DcMotorEx) Right).setTargetPositionTolerance(20);
      Left.setTargetPosition(leftTarget);
      Right.setTargetPosition(rightTarget);
      Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Left.setPower(power);
      Right.setPower(power);
      leftList = JavaUtil.createListWith();
      rightList = JavaUtil.createListWith();
      while (opModeIsActive() && (Left.isBusy() || Right.isBusy())) {
        leftList.add(Left.getCurrentPosition());
        rightList.add(Right.getCurrentPosition());
      }
      ((DcMotorEx) Left).setTargetPositionTolerance(5);
      ((DcMotorEx) Right).setTargetPositionTolerance(5);
      sleep(200);
      Left.setTargetPosition(leftTarget);
      Right.setTargetPosition(rightTarget);
      Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // Writes motor pos values to debug log for testing
      while (opModeIsActive() && (Left.isBusy() || Right.isBusy())) {
        leftList.add(Left.getCurrentPosition());
        rightList.add(Right.getCurrentPosition());
      }
      RobotLog.ii("DbgLog", "Left Target=" + leftTarget);
      RobotLog.ii("DbgLog", "Right Target=" + rightTarget);
      RobotLog.ii("DbgLog", "leftList=");
      RobotLog.ii("DbgLog", JavaUtil.makeTextFromList(leftList, ","));
      RobotLog.ii("DbgLog", "rightList=");
      RobotLog.ii("DbgLog", JavaUtil.makeTextFromList(rightList, ","));
      Left.setPower(0);
      Right.setPower(0);
    }
  }

  /**
   * drives a specified distance
   */
  private void drive(double power, double leftInches, double rightInches) {
    if (opModeIsActive()) {
      rightTarget = (int) (Right.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN);
      leftTarget = (int) (Left.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ((DcMotorEx) Left).setTargetPositionTolerance(20);
      ((DcMotorEx) Right).setTargetPositionTolerance(20);
      Left.setTargetPosition(leftTarget);
      Right.setTargetPosition(rightTarget);
      Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Left.setPower(power);
      Right.setPower(power);
      leftList = JavaUtil.createListWith();
      rightList = JavaUtil.createListWith();
      while (opModeIsActive() && (Left.isBusy() || Right.isBusy())) {
        leftList.add(Left.getCurrentPosition());
        rightList.add(Right.getCurrentPosition());
      }
      ((DcMotorEx) Left).setTargetPositionTolerance(5);
      ((DcMotorEx) Right).setTargetPositionTolerance(5);
      sleep(200);
      Left.setTargetPosition(leftTarget);
      Right.setTargetPosition(rightTarget);
      Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (opModeIsActive() && (Left.isBusy() || Right.isBusy())) {
        leftList.add(Left.getCurrentPosition());
        rightList.add(Right.getCurrentPosition());
      }
      RobotLog.ii("DbgLog", "Left Target=" + leftTarget);
      RobotLog.ii("DbgLog", "Right Target=" + rightTarget);
      RobotLog.ii("DbgLog", "leftList=");
      RobotLog.ii("DbgLog", JavaUtil.makeTextFromList(leftList, ","));
      RobotLog.ii("DbgLog", "rightList=");
      RobotLog.ii("DbgLog", JavaUtil.makeTextFromList(rightList, ","));
      Left.setPower(0);
      Right.setPower(0);
    }
  }

  /**
   * lifts arm to level
   */
  private void liftArm(int toLevel) {
    if (opModeIsActive()) {
      if (toLevel == 1) {
        elbowTarget = 342;
        armTarget = 2365;
        telemetry.speak("1", null, null);
      } else if (toLevel == 2) {
        elbowTarget = 170;
        armTarget = 50;
        telemetry.speak("2", null, null);
      } else {
        elbowTarget = 610;
        armTarget = 2500;
        telemetry.speak("3", null, null);
      }
      ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      ArmMotor.setTargetPosition(armTarget);
      ElbowMotor.setTargetPosition(elbowTarget);
      ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmMotor.setPower(1);
      ElbowMotor.setPower(0.4);
      while (opModeIsActive() && (ElbowMotor.isBusy() || ArmMotor.isBusy())) {
      }
      sleep(1000);
      Left.setPower(0);
      Right.setPower(0);
    }
  }

  /**
   * vuforia print
   */
  private void displayInfo(int i) {
    // Display label info.
    // Display the label and index number for the recognition.
    telemetry.addData("label " + i, recognition.getLabel());
    // Display upper corner info.
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
    // Display lower corner info.
    // Display the location of the bottom right corner
    // of the detection boundary for the recognition
    telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
  }

  /**
   * goes one foot forward
   */
  private void drive1Foot() {
    drive(1, 12, 12);
  }

  /**
   * 360 turn
   */
  private void rotate360degClockwise() {
    drive(1, 39.5, -39.5);
  }

  /**
   * Reads the barcode by checking data from sensors
   */
  private int readBarcode() {
    boolean isLeftFound;
    boolean isRightFound;

    for (int count = 0; count < 10; count++) {
      if (RearLeftOuter.getDistance(DistanceUnit.CM) < 800) {
        isLeftFound = true;
      }
      if (RearRightOuter.getDistance(DistanceUnit.CM) < 800) {
        isRightFound = true;
      }
      sleep(100);
    }
    if (isLeftFound) {
      level = 3;
    } else if (isRightFound) {
      level = 2;
    } else {
      level = 1;
    }
    return level;
  }

  /**
   * Backs up a different distance depending on the level
   */
  private void backUp(
      // TODO: Enter the type for argument named toLevel
      UNKNOWN_TYPE toLevel) {
    if (level == 1) {
      drive(1, -2.5, -2.5);
    } else if (level == 2) {
      drive(1, -6.1, -6.1);
    } else {
      drive(1, -1, -1);
    }
  }

  /**
   * depending on the level it goes a different distance forward
   */
  private void driveToHub(int toLevel) {
    if (level == 1) {
      drive(0.6, 6.9, 6.9);
    } else if (level == 2) {
      drive(0.6, 12.6, 12.6);
    } else {
      drive(0.6, 5.6, 5.6);
    }
  }

  /**
   * Drops cube
   */
  private void dropCube() {
    Servo1.setPosition(1);
    sleep(2000);
    Servo1.setPosition(0);
  }

  /**
   * Turns robot 135 degrees counter clockwise
   */
  private void turn135CCW() {
    drive(1, -17, 17);
  }

  /**
   * turns robot 45 degrees clockwise
   */
  private void turn45CW() {
    drive(0.7, 6.3, -6.3);
  }
}
