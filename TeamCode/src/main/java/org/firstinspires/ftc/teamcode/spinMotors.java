package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "spinMotors", group = "Joystick Opmode")
public class spinMotors extends OpMode {

    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if (gamepad1.a){
            aDrive.setPower(0.5);
        }else{
            aDrive.setPower(0);
        }

        if (gamepad1.b){
            bDrive.setPower(-0.5);
        }else{
            bDrive.setPower(0);
        }

        if (gamepad1.x){
            cDrive.setPower(-0.5);
        }else{
            cDrive.setPower(0);
        }

        if (gamepad1.y){ dDrive.setPower(0.5); }
        else{ dDrive.setPower(0); }

    }

}
