#!/usr/bin/env node
import assert from "node:assert/strict";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import { pathToFileURL } from "node:url";

import { hostModuleHref, PACKAGE_ROOT } from "./paths.mjs";

async function fileExists(candidate) {
    try {
        const stat = await fs.lstat(candidate);
        return stat.isFile() && !stat.isSymbolicLink();
    } catch (error) {
        if (error.code === "ENOENT") return false;
        throw error;
    }
}

export async function verifyDynamicsPackage() {
    const { buildLocalPlantPackage } = await import(hostModuleHref("server/plugins/plantBuild.js"));
    const { PluginStore } = await import(hostModuleHref("server/storage/PluginStore.js"));
    const { verifyPluginPackage } = await import(hostModuleHref("app/plugin/PluginPackage.js"));

    const document = JSON.parse(await fs.readFile(path.join(PACKAGE_ROOT, "plugin.json"), "utf8"));
    const temp = await fs.mkdtemp(path.join(os.tmpdir(), "dynamics-verify-"));
    const store = new PluginStore(temp);
    let packageDirectory = null;
    try {
        const built = await buildLocalPlantPackage({
            sourceDirectory: PACKAGE_ROOT,
            directoryKey: "dynamics/package",
            storeRoot: store.root,
            document,
        });
        packageDirectory = built.packageDirectory;
        const packed = await fs.readdir(packageDirectory);
        assert.deepEqual(packed.sort(), ["plant.wasm", "plugin.json"]);
        const metadata = await store.installFromDirectory(packageDirectory);
        assert.equal(metadata.pluginId, "vendor.cornellev.dynamics");
        assert.equal(metadata.version, "0.1.0");
        const verified = verifyPluginPackage(await store.getPackage(metadata.packageHash));
        assert.equal(verified.document.id, "vendor.cornellev.dynamics");
        assert.equal(verified.document.plants[0].id, "vendor.cornellev.dynamics.bicycle");
        assert.deepEqual([...verified.document.capabilities], ["plant.step"]);
        assert.equal(verified.document.units.length, 0);
        assert.equal(verified.document.systems.length, 0);
        assert.equal(await fileExists(path.join(PACKAGE_ROOT, "plant.wasm")), false);
        return metadata;
    } finally {
        if (packageDirectory) await fs.rm(packageDirectory, { recursive: true, force: true });
        await fs.rm(temp, { recursive: true, force: true });
    }
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) {
    const metadata = await verifyDynamicsPackage();
    process.stdout.write(`${JSON.stringify({ ok: true, ...metadata }, null, 2)}\n`);
}
