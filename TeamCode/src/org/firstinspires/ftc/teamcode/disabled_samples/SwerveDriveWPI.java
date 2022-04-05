package org.firstinspires.ftc.teamcode.disabled_samples;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SwerveDriveWPI {
    double width = 16*0.0254;//in meters
    double length = 14*0.0254;//in meters
    public static final double WHEEL_CIRCUMFERENCE = (4*0.0254) * Math.PI;
    public static final double TICKS_PER_ROTATION = 1120;
    public static final double MAX_TICKS_PER_SECOND = 2500;

    public static final double CHASSIS_RAD_SQUARED = (7*.0254)*(7*.0254) + (8*.0254)*(8*.0254);
    public static final double MAX_DRIVE_SPEED = MAX_TICKS_PER_SECOND * WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / Math.sqrt(CHASSIS_RAD_SQUARED);


    CRServo[] crServos = new CRServo[4];    //Steering
    DcMotor[] encoders = new DcMotor[4];    //Encoders to monitor steering
    DcMotor[] motors = new DcMotor[4];
    Translation2d m_frontLeftLocation =new Translation2d(width/2, length/2);
    Translation2d m_frontRightLocation =new Translation2d(width/2, -length/2);
    Translation2d m_backLeftLocation =new Translation2d(-width/2, length/2);
    Translation2d m_backRightLocation =new Translation2d(-width/2, -length/2);
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

public void drive(double vx, double vy, double va, double imu){
    if(Math.abs(va)>0){ // manual offset
        double dx = vx;
        vx = vx * Math.cos(-Math.PI/9) - vy * Math.sin(-Math.PI/9);
        vy = vy * Math.cos(-Math.PI/9) + dx * Math.sin(-Math.PI/9);
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, va, Rotation2d.fromRadians(imu)
    );

// Convert to module states
    SwerveModuleState[] moduleStates =
            m_kinematics.toSwerveModuleStates(speeds);

// Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

// Front right module state
    SwerveModuleState frontRight = moduleStates[1];

// Back left module state
    SwerveModuleState backLeft = moduleStates[2];

// Back right module state
    SwerveModuleState backRight = moduleStates[3];
    backLeft = backLeft.optimize(backLeft,Rotation2d.fromRadians(getSteerRadians(0)));
    frontLeft = frontLeft.optimize(frontLeft,Rotation2d.fromRadians(getSteerRadians(1)));
    frontRight = frontRight.optimize(frontRight,Rotation2d.fromRadians(getSteerRadians(2)));
    backRight = backRight.optimize(backRight,Rotation2d.fromRadians(getSteerRadians(3)));
    motors[0].setPower(backLeft.speedMetersPerSecond/MAX_DRIVE_SPEED);
    motors[1].setPower(frontLeft.speedMetersPerSecond/MAX_DRIVE_SPEED);
    motors[2].setPower(frontRight.speedMetersPerSecond/MAX_DRIVE_SPEED);
    motors[3].setPower(backRight.speedMetersPerSecond/MAX_DRIVE_SPEED);
    setSteer(0,backLeft.angle.getRadians());
    setSteer(1,frontLeft.angle.getRadians());
    setSteer(2,frontRight.angle.getRadians());
    setSteer(3,backRight.angle.getRadians());
}

    public void init(HardwareMap hardwareMap) {
        String[] crServoNames = new String[]{"back_left_crservo", "front_left_crservo", "front_right_crservo", "back_right_crservo"};
        for (int i = 0; i < 4; i++)
            crServos[i] = (CRServo) hardwareMap.get(CRServo.class, crServoNames[i]);
        String[] encoderNames = new String[]{"back_left_encoder", "front_left_encoder", "front_right_encoder", "back_right_encoder"};
        for (int i = 0; i < 4; i++)
            encoders[i] = (DcMotor) hardwareMap.get(DcMotor.class, encoderNames[i]);
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i = 0; i < 4; i++)
            motors[i] = (DcMotor) hardwareMap.get(DcMotor.class, motorNames[i]);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private double getSteerRadians(int i){
        return normalizeRadians(2.0 * Math.PI * encoders[i].getCurrentPosition()/TICKS_PER_ROTATION);
    }
    private void setSteer(int i, double targetSteer){
        double currentSteer = getSteerRadians(i);
        double offset = normalizeRadians(targetSteer - currentSteer);
        double steerPower = Range.clip(4.0*offset/Math.PI, -1, 1);
        crServos[i].setPower(steerPower);
    }
    public static double normalizeRadians(double radians) {
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }


}
