export const TAPE_KIND = "cev-sim.bicycle-tape";
export const TAPE_VERSION = 1;

const BIG_CAR_STEPS = [
    { dt: 0.02, acceleration: 0.4, steering: 0.05 },
    { dt: 0.02, acceleration: -0.2, steering: -0.1 },
    { dt: 0.02, acceleration: 1, steering: 0.2 },
    { dt: 0.02, acceleration: 0, steering: 0 },
    { dt: 0.02, acceleration: 0.25, steering: 0.15 },
    { dt: 0.02, acceleration: -0.5, steering: -0.05 },
];

export function tapeCases(bigCarKinematics) {
    return [
        {
            id: "big-car",
            wheelbase: bigCarKinematics.wheelbase,
            maxSteeringAngle: bigCarKinematics.maxSteeringAngle,
            initial: {
                position: [1, 0, 2],
                rotation: [0, 0.2, 0],
                velocity: [3, 0, 0],
                acceleration: [1, 0, 0],
                steering: 0.1,
            },
            steps: BIG_CAR_STEPS,
        },
        {
            id: "defaults",
            wheelbase: 1.5,
            maxSteeringAngle: 0.6,
            initial: {
                position: [0, 2, 0],
                rotation: [0.15, -0.4, 0.05],
                velocity: [2, 0, 0],
                acceleration: [0, 0, 0],
                steering: 0,
            },
            steps: [
                { dt: 0.05, acceleration: 0, steering: 0.8 },
                { dt: 0.05, acceleration: -1, steering: -0.3 },
                { dt: 0.1, acceleration: 0.5, steering: 0 },
            ],
        },
    ];
}
