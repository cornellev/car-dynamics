import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";
import { promisify } from "node:util";

import { simulateCase } from "../tools/bicycle-host.mjs";
import { FIXTURE_PATH, hostModuleHref, PACKAGE_ROOT } from "../tools/paths.mjs";

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

function packState(sample) {
    const state = new Float64Array(13);
    state.set(sample.initial.position, 0);
    state.set(sample.initial.rotation, 3);
    state.set(sample.initial.velocity, 6);
    state.set(sample.initial.acceleration, 9);
    state[12] = sample.initial.steering;
    return state;
}

async function stepWasm(stepper, sample) {
    let state = packState(sample);
    return sample.steps.map((step) => {
        state[9] = step.acceleration;
        state[12] = step.steering;
        state = stepper.step(state, step.dt);
        return {
            x: state[0],
            z: state[2],
            yaw: state[4],
            velocityX: state[6],
        };
    });
}

test("wasm bicycle matches the javascript plant within 1e-9", async () => {
    const bytes = await buildWasm();
    const tape = JSON.parse(await fs.readFile(FIXTURE_PATH, "utf8"));
    const document = JSON.parse(await fs.readFile(path.join(PACKAGE_ROOT, "plugin.json"), "utf8"));
    const plant = document.plants[0];
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
    for (const sample of tape.cases) {
        const javascript = await simulateCase(sample);
        const stepper = createPlantStepper(compiled.get(plant.id), definition, {
            kinematics: {
                wheelbase: sample.wheelbase,
                maxSteeringAngle: sample.maxSteeringAngle,
            },
        });
        const wasm = await stepWasm(stepper, sample);
        assert.equal(wasm.length, sample.steps.length);
        for (let index = 0; index < wasm.length; index += 1) {
            for (const field of ["x", "z", "yaw", "velocityX"]) {
                assert.ok(
                    Math.abs(wasm[index][field] - javascript[index][field]) <= 1e-9,
                    `${sample.id} step ${index} ${field}`,
                );
                assert.ok(
                    Math.abs(javascript[index][field] - sample.steps[index].expected[field]) <= 1e-9,
                    `${sample.id} step ${index} ${field} drifted from the fixture`,
                );
            }
        }
    }
});
