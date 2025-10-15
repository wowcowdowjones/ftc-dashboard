package org.firstinspires.ftc.robotcontroller.internal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AprilTag Auto Dashboard Fixed", group = "Autonomous")
public class AprilTag extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        // Start dashboard camera stream
        dashboard.startCameraStream(limelight, 0);
    }

    @Override
    public void start() {
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();
        boolean hasResult = llResult != null;
        boolean hasTarget = hasResult && llResult.isValid(); // valid AprilTag detected

        telemetry.addData("Has Result?", hasResult);
        telemetry.addData("Valid Target?", hasTarget);

        TelemetryPacket packet = new TelemetryPacket();

        if (hasTarget) {
            Pose3D botPose = llResult.getBotpose();
            double distance = getDistanceFromTag(llResult.getTa()); // fixed name
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());

            // Approximate bounding box using Tx, Ty, and Ta
            double centerX = llResult.getTx() / 29.8; // normalize horizontal FOV
            double centerY = -llResult.getTy() / 23.8; // normalize vertical FOV
            double size = Math.sqrt(llResult.getTa() / 100.0); // approximate size from area

            // Draw rectangle with red stroke
            packet.fieldOverlay()
                    .setStroke("#FF0000") // red
                    .strokeRect(centerX - size / 2.0, centerY - size / 2.0,
                            centerX + size / 2.0, centerY + size / 2.0);

        } else {
            telemetry.addLine("No valid AprilTag detected");
        }

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    // ✅ Correct method placement and variable naming
    private double getDistanceFromTag(double ta) {
        double scale = 30665.95;  // corrected spelling
        if (ta <= 0) return Double.POSITIVE_INFINITY; // avoid divide-by-zero
        return scale / ta;
    }
}
