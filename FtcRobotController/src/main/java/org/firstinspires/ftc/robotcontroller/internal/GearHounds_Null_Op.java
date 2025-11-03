//package org.firstinspires.ftc.robotcontroller.internal;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
//
//
//@Config
//@TeleOp(name = "GearHounds NullOp", group = "Concept")
//public class GearHounds_Null_Op extends OpMode {
//    private final GearhoundsHardware robot = new GearhoundsHardware();
//    private ElapsedTime runtime = new ElapsedTime();
//    private FtcDashboard dashboard;
////  public static double "variable here"
//
//    /**
//     * This method will be called once, when the INIT button is pressed.
//     */
//    @Override
//    public void init() {
//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//        robot.init(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//    }
//
//    /**
//     * This method will be called repeatedly during the period between when
//     * the INIT button is pressed and when the START button is pressed (or the
//     * OpMode is stopped).
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /**
//     * This method will be called once, when the START button is pressed.
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    /**
//     * This method will be called repeatedly during the period between when
//     * the START button is pressed and when the OpMode is stopped.
//     */
//    @Override
//    public void loop() {
////        TelemetryPacket packet = new TelemetryPacket();
////        packet.put("ExampleName", DataHere);
////        dashboard.sendTelemetryPacket(packet);
//    }
//
//    /**
//     * This method will be called once, when this OpMode is stopped.
//     * <p>
//     * Your ability to control hardware from this method will be limited.
//     */
//    @Override
//    public void stop() {
//
//    }
//}
//
