# Kinematic bicycle dynamics plugin

Independent development kit for a cev-sim plant ABI 1 plugin. The installed
package is **`package/` only**. This repository lives outside cev-sim’s tracked
tree.

The plant clones the built-in explicit-Euler bicycle: accelerate, translate in
`x`/`z` at the old yaw, then integrate yaw. The host still drapes `y` and pitch
after a successful step. Selecting this plant does not replace the built-in
JavaScript bicycle unless a vehicle sets `dynamics.plant`.

## Layout

```text
package/          # The only directory passed to Install. Cargo builds plant.wasm.
crates/tape/      # Native replay of fixtures/bicycle-tape.v1.json through step.
fixtures/         # Committed bicycle tape, generated from the JavaScript plant.
tools/            # Export, path lookup, and install verify. Never packed.
tests/            # WebAssembly parity against the JavaScript plant.
```

Plugin id: `vendor.cornellev.dynamics`. Plant id:
`vendor.cornellev.dynamics.bicycle`.

## Authoring

1. Install `plugins/dynamics/package` from the cev-sim plugin library. Install
   runs `cargo build --release --lib --locked --target wasm32-unknown-unknown`
   and packs `plugin.json` plus `plant.wasm`. It does not modify this tree.
2. On the vehicle, set `dynamics.plant` to `vendor.cornellev.dynamics.bicycle`.
3. Grant `plant.step` on the run. The matching vehicle lock lists that plant.

Parameters are `kinematics.wheelbase` and `kinematics.maxSteeringAngle`, in
that order. A fresh WebAssembly instance is created on reset.

## Develop

From this kit. Tooling walks parent directories until it finds
`app/plugin/PluginPackage.js`. Set `CEV_SIM_ROOT` when the checkout lives
elsewhere.

```bash
npm run export:tape
npm test
npm run verify
```

`npm test` runs the plant crate tests, the native tape replay, and a
WebAssembly comparison within 1e-9 of `KinematicVehiclePlant`. `npm run verify`
builds into a temporary plugin store and checks the installed document. Rust
1.81.0 with the `wasm32-unknown-unknown` target is selected by
`rust-toolchain.toml`.
