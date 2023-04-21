package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

public class DriveTrain extends SubsystemBase{
    
     // gyro calibration constant, may need to be adjusted;
    // gyro value of 360 is set to correspond to one full revolution
    AHRS ahrs;
    private static final int kFrontLeftChannel = 0;
    private static final int kRearLeftChannel = 1;
    private static final int kFrontRightChannel = 2;
    private static final int kRearRightChannel = 3;
    private MecanumDrive robotdrive;

    public DriveTrain(){
        Spark frontLeft = new Spark(kFrontLeftChannel);
        Spark rearLeft = new Spark(kRearLeftChannel);
        Spark frontRight = new Spark(kFrontRightChannel);
        Spark rearRight = new Spark(kRearRightChannel);

        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        frontRight.setInverted(true);
        rearRight.setInverted(true);

        robotdrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

       

        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            ahrs = new AHRS(I2C.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        ahrs.calibrate();
        
        ahrs.reset();
        
    }

    public void DriveMecanum(double y, double x, double z){
        robotdrive.driveCartesian(
            y*.9,
            x*.9,
            z,
           ahrs.getRotation2d().times(-1)
           );
    }
}
