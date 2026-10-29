import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";
import { promisify } from "node:util";

import { hostModuleHref, PACKAGE_ROOT } from "../tools/paths.mjs";

const execFileAsync = promisify(execFile);

function selectWasm(stdout, packageDir) {
    const artifacts = [];
    for (const line of String(stdout ?? "").split("\n")) {
        if (!line.trim().startsWith("{")) continue;
        let message;
        try {
            message = JSON.parse(line);
        } catch {
            continue;
        }
        if (message.reason !== "compiler-artifact") continue;
        const srcPath = message.target?.src_path || "";
        if (srcPath) {
            const relative = path.relative(packageDir, srcPath);
            if (relative.startsWith("..") || path.isAbsolute(relative)) continue;
        }
        for (const filename of message.filenames ?? []) {
            if (typeof filename === "string" && filename.endsWith(".wasm")) artifacts.push(filename);
        }
    }
    const artifact = artifacts.at(-1);
    if (!artifact) throw new Error("Cargo produced no plant WebAssembly artifact.");
    return artifact;
}

async function buildWasm() {
    const targetDir = await fs.mkdtemp(path.join(os.tmpdir(), "dynamics-wasm-"));
    try {
        const { stdout } = await execFileAsync("cargo", [
            "build",
            "--release",
            "--lib",
            "--locked",
            "--target",
            "wasm32-unknown-unknown",
            "--message-format=json",
        ], {
            cwd: PACKAGE_ROOT,
            env: { ...process.env, CARGO_TARGET_DIR: targetDir },
            maxBuffer: 16 * 1024 * 1024,
        });
        const artifact = selectWasm(stdout, PACKAGE_ROOT);
        return new Uint8Array(await fs.readFile(artifact));
    } finally {
        await fs.rm(targetDir, { recursive: true, force: true });
    }
}

test("wasm axle forces match the javascript contact plant", async () => {
    const bytes = await buildWasm();
    const document = JSON.parse(await fs.readFile(path.join(PACKAGE_ROOT, "plugin.json"), "utf8"));
    const plant = document.plants[0];
    assert.equal(plant.model, "contact-bicycle-v1");
    const { axleContactForces } = await import(
        hostModuleHref("app/simulation/vehicles/contactBicycle.js")
    );
    const { compilePlantModules, createPlantStepper } = await import(
        hostModuleHref("app/simulation/vehicles/WasmPlantStepper.js")
    );
    const definition = {
        id: plant.id,
        parameters: plant.parameters,
        ownership: { pluginId: document.id },
        moduleBytes: bytes,
    };
    const compiled = await compilePlantModules(new Map([[plant.id, definition]]));
    const stepper = createPlantStepper(compiled.get(plant.id), definition, {
        kinematics: { wheelbase: 1.5, maxSteeringAngle: 0.6 },
    });
    const state = new Float64Array(13);
    state[0] = 3;
    state[1] = 4;
    state[2] = 5;
    state[4] = 0.25;
    state[6] = 8;
    state[9] = 8;
    state[10] = 1;
    state[11] = 1500 * 9.81;
    state[12] = 0.4;
    const next = stepper.step(state, 0.01);
    const forces = axleContactForces({
        dt: 0.01,
        vx: 8,
        vy: 0,
        r: 0,
        delta: 0.4,
        axCommand: 8,
        gradeX: 0,
        normalLoad: 1500 * 9.81,
        inContact: true,
        lf: 0.75,
        lr: 0.75,
    });
    for (const [actual, expected, label] of [
        [next[0], 3, "x"],
        [next[6], forces.Fxf, "Fxf"],
        [next[7], forces.Fyf, "Fyf"],
        [next[9], forces.Fxr, "Fxr"],
        [next[10], forces.Fyr, "Fyr"],
    ]) {
        assert.ok(Math.abs(actual - expected) <= 1e-6, `${label}: ${actual} vs ${expected}`);
    }
    state[10] = 0;
    state[11] = 0;
    const airborne = stepper.step(state, 0.02);
    assert.equal(airborne[6], 0);
    assert.equal(airborne[7], 0);
    assert.equal(airborne[9], 0);
    assert.equal(airborne[10], 0);
    assert.equal(airborne[1], 4);

    for (const vx of [-2, -0.001, 0, 0.001, 2]) {
        for (const vy of [-1, -0.0001, 0, 0.0001, 1]) {
            for (const delta of [-0.2, 0, 0.2]) {
                for (const dt of [0.01, 1 / 60, 0.1]) {
                    state[6] = vx;
                    state[7] = vy;
                    state[8] = 0.1;
                    state[9] = 0;
                    state[10] = 1;
                    state[11] = 1500 * 9.81;
                    state[12] = delta;
                    const actual = stepper.step(state, dt);
                    const expected = axleContactForces({ vx, vy, r: 0.1, delta, dt, normalLoad: state[11] });
                    for (const [index, key] of [[6, "Fxf"], [7, "Fyf"], [9, "Fxr"], [10, "Fyr"]]) {
                        assert.ok(Math.abs(actual[index] - expected[key]) <= 1e-6, `${vx}, ${vy}, ${delta}, ${dt}: ${key}`);
                    }
                }
            }
        }
    }

    const { replayContactRamp } = await import(hostModuleHref("tests/helpers/contactRampRegression.js"));
    const { normalizeVehicleManifest } = await import(hostModuleHref("app/vehicles/VehicleManifest.js"));
    const fixture = JSON.parse(await fs.readFile(new URL(hostModuleHref("tests/fixtures/headless/contact-ramp-regression.v1.json")), "utf8"));
    const manifest = normalizeVehicleManifest({ ...fixture.vehicle, dynamics: { plant: plant.id } });
    const createStepper = () => createPlantStepper(compiled.get(plant.id), definition, manifest);
    const numeric = await replayContactRamp({ manifest, stepper: createStepper() });
    const browser = await replayContactRamp({ manifest, stepper: createStepper(), browser: true });
    assert.ok(numeric.maxLateralSpeed < 3, `sideways speed ${numeric.maxLateralSpeed}`);
    assert.ok(numeric.floorSteps > 0);
    for (let index = 0; index < numeric.states.length; index += 1) {
        assert.deepEqual(numeric.states[index], browser.states[index]);
    }
    const final = numeric.states.at(-1);
    assert.ok(Math.hypot(final.velocity.x, final.lateralVelocity) < 0.01);

    const stopping = JSON.parse(await fs.readFile(new URL(hostModuleHref("tests/fixtures/headless/contact-stopping-regression.v1.json")), "utf8"));
    const stopManifest = normalizeVehicleManifest({ ...stopping.vehicle, dynamics: { plant: plant.id } });
    const stopOptions = { fixture: stopping, releaseSeconds: 15 };
    const wasm = await replayContactRamp({ ...stopOptions, manifest: stopManifest, stepper: createPlantStepper(compiled.get(plant.id), definition, stopManifest) });
    const browserWasm = await replayContactRamp({ ...stopOptions, manifest: stopManifest, stepper: createPlantStepper(compiled.get(plant.id), definition, stopManifest), browser: true });
    assert.deepEqual(wasm.states, browserWasm.states);
    // The package ABI uses half-wheelbase axles, not the authored asymmetric wheel centers.
    const half = stopManifest.kinematics.wheelbase / 2;
    const reference = await replayContactRamp({ ...stopOptions, axle: { lf: half, lr: half } });
    for (let index = 0; index < wasm.states.length; index += 1) {
        const actual = wasm.states[index];
        const expected = reference.states[index];
        for (const field of ["position", "rotation", "velocity", "acceleration"]) for (const axis of ["x", "y", "z"]) {
            assert.ok(Math.abs(actual[field][axis] - expected[field][axis]) <= 1e-9, `${index}: ${field}.${axis}`);
        }
        for (const field of ["lateralVelocity", "yawRate", "verticalVelocity", "axCommand"]) {
            assert.ok(Math.abs(actual[field] - expected[field]) <= 1e-9, `${index}: ${field}`);
        }
    }
    const rest = wasm.states.slice(-Math.floor(10e9 / stopping.stepNs));
    for (const state of rest) {
        assert.ok(Math.hypot(state.velocity.x, state.lateralVelocity) < 1e-3);
        assert.ok(Math.hypot(state.position.x - rest[0].position.x, state.position.z - rest[0].position.z) < 1e-3);
    }

    for (const dt of [0, 0.01, 1 / 60, 0.05, 0.1]) for (const vx of [-0.0011, 0, 0.0011]) for (const gradeX of [-1, 0, 1]) {
        state[3] = gradeX;
        state[6] = vx;
        state[7] = 0;
        state[8] = 0;
        state[9] = 0.01;
        state[12] = 0;
        const actual = stepper.step(state, dt);
        const expected = axleContactForces({ vx, dt, axCommand: 0.01, gradeX, normalLoad: state[11] });
        for (const [slot, key] of [[6, "Fxf"], [7, "Fyf"], [9, "Fxr"], [10, "Fyr"]]) {
            assert.ok(Math.abs(actual[slot] - expected[key]) <= 1e-9, `${dt}, ${vx}, ${gradeX}: ${key}`);
        }
    }
});
