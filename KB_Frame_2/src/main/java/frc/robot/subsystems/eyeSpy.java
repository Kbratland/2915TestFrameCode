// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class eyeSpy extends SubsystemBase {
//   PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
//   private double camHightM = .55;
//   private double targetHightM = 1;
//   private double camPtichRad = 0;

//   public boolean hasTarget = false;
//   public double pitch = 0;
//   public double yaw = 0;
//   public double distanceMeters = 0;
//   private double area = 0;
//   public double getTargetYaw()
//   {
//     return(this.yaw);
//   }
//   public double getTargetPitch()
//   {
//     return(this.pitch);
//   }
//   public double distance()
//   {
//     distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(camHightM, targetHightM, camPtichRad, pitch);
//     return(distanceMeters);
//   }
//   @Override
//   public void periodic()
//   {
//     var result = this.photonCamera.getLatestResult();
//     if(result.hasTargets())
//     {
//       this.pitch = result.getBestTarget().getPitch();
//       this.yaw = result.getBestTarget().getYaw();
//       this.area = result.getBestTarget().getArea();
//     }
//   }
// }
