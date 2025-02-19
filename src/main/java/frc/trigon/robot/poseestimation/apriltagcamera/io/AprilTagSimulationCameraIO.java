package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagSimulationCameraIO extends AprilTagPhotonCameraIO {
    private final PhotonCameraSim cameraSimulation;

    public AprilTagSimulationCameraIO(String cameraName) {
        super(cameraName);

        cameraSimulation = new PhotonCameraSim(photonCamera, AprilTagCameraConstants.SIMULATION_CAMERA_PROPERTIES);
        cameraSimulation.enableDrawWireframe(true);
    }

    @Override
    protected void addSimulatedCameraToVisionSimulation(Transform3d robotCenterToCamera) {
        AprilTagCameraConstants.VISION_SIMULATION.addCamera(cameraSimulation, robotCenterToCamera);
    }
}