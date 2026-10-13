import { hostModuleHref } from "./paths.mjs";

export async function simulateCase(sample) {
    const { createVehiclePlantDefinition, KinematicVehiclePlant } = await import(
        hostModuleHref("app/simulation/vehicles/KinematicVehiclePlant.js")
    );
    const { getBuiltInVehicleManifest } = await import(
        hostModuleHref("app/vehicles/BuiltInVehicleManifests.js")
    );
    const manifest = structuredClone(getBuiltInVehicleManifest("big-car"));
    delete manifest.dynamics;
    manifest.kinematics = {
        ...manifest.kinematics,
        wheelbase: sample.wheelbase,
        maxSteeringAngle: sample.maxSteeringAngle,
    };
    const [px, py, pz] = sample.initial.position;
    const [rx, ry, rz] = sample.initial.rotation;
    const [vx, vy, vz] = sample.initial.velocity;
    const [ax, ay, az] = sample.initial.acceleration;
    const plant = new KinematicVehiclePlant(createVehiclePlantDefinition({
        id: sample.id,
        type: "big-car",
        pose: {
            position: { x: px, y: py, z: pz },
            rotation: { x: rx, y: ry, z: rz, order: "XYZ" },
        },
        linearVelocity: { x: vx, y: vy, z: vz },
        linearAcceleration: { x: ax, y: ay, z: az },
        steeringAngle: sample.initial.steering,
    }, { manifest }));
    return sample.steps.map((step) => {
        plant.acceleration.x = step.acceleration;
        plant.steeringAngle = step.steering;
        plant.update(step.dt);
        return {
            x: plant.position.x,
            z: plant.position.z,
            yaw: plant.rotation.y,
            velocityX: plant.velocity.x,
        };
    });
}
