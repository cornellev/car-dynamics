import path from "node:path";
import { existsSync } from "node:fs";
import { fileURLToPath, pathToFileURL } from "node:url";

export const KIT_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
export const PACKAGE_ROOT = path.join(KIT_ROOT, "package");
export const FIXTURE_PATH = path.join(KIT_ROOT, "fixtures", "bicycle-tape.v1.json");

export function hostRoot() {
    if (process.env.CEV_SIM_ROOT) return path.resolve(process.env.CEV_SIM_ROOT);
    let current = KIT_ROOT;
    while (current !== path.dirname(current)) {
        if (existsSync(path.join(current, "app/plugin/PluginPackage.js"))) return current;
        current = path.dirname(current);
    }
    throw new Error("Set CEV_SIM_ROOT to the cev-sim repository root.");
}

export function hostModuleHref(relativePath) {
    return pathToFileURL(path.join(hostRoot(), relativePath)).href;
}
