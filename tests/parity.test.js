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
});
