#!/usr/bin/env node
import { promises as fs } from "node:fs";
import path from "node:path";
import { pathToFileURL } from "node:url";

import { simulateCase } from "./bicycle-host.mjs";
import { TAPE_KIND, TAPE_VERSION, tapeCases } from "./cases.mjs";
import { FIXTURE_PATH, hostModuleHref } from "./paths.mjs";

export async function exportBicycleTape() {
    const { getBuiltInVehicleManifest } = await import(
        hostModuleHref("app/vehicles/BuiltInVehicleManifests.js")
    );
    const bigCar = getBuiltInVehicleManifest("big-car");
    const cases = [];
    for (const sample of tapeCases(bigCar.kinematics)) {
        const expected = await simulateCase(sample);
        cases.push({
            ...sample,
            steps: sample.steps.map((step, index) => ({ ...step, expected: expected[index] })),
        });
    }
    const document = { kind: TAPE_KIND, version: TAPE_VERSION, cases };
    await fs.mkdir(path.dirname(FIXTURE_PATH), { recursive: true });
    await fs.writeFile(FIXTURE_PATH, `${JSON.stringify(document, null, 2)}\n`);
    return document;
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) {
    const document = await exportBicycleTape();
    process.stdout.write(`${document.cases.length} cases -> ${FIXTURE_PATH}\n`);
}
