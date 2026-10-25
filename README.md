# Contact bicycle dynamics plugin

Independent development kit for a cev-sim plant ABI 1 plugin. The installed
package is **`package/` only**. This repository lives outside cev-sim’s tracked
tree.

The plant reports front and rear axle forces for `contact-bicycle-v1`. The
simulator adds gravity and moves the car. The Zenoh horizon still integrates
the planar bicycle and does not fall. Selecting this plant replaces the
built-in car when a vehicle sets `dynamics.plant`.

## Layout

```text
package/          # The only directory passed to Install. Cargo builds plant.wasm.
crates/tape/      # Native replay of fixtures/bicycle-tape.v1.json through kinematic_step.
crates/predict/   # One call, or a constant-command horizon, through kinematic_step.
crates/wire/      # 136-byte little-endian request and response frames.
crates/zenoh/     # dynamics-zenoh queryable. Not packed and not built by Install.
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

Parameters are wheelbase, max steering, then mass, yaw inertia, front and rear
cornering stiffness, friction, and rolling resistance. The last six use the
built-in defaults when omitted. A fresh WebAssembly instance is created on reset.

## Develop

From this kit. Tooling walks parent directories until it finds
`app/plugin/PluginPackage.js`. Set `CEV_SIM_ROOT` when the checkout lives
elsewhere.

```bash
npm run export:tape
npm test
npm run verify
```

`npm test` runs the plant crate tests, the native tape replay, the prediction
and wire crates, the Zenoh loopback query, and a WebAssembly comparison of
axle forces against the JavaScript contact plant. `npm run verify` builds into a temporary
plugin store and checks the installed document. Install uses Rust 1.81.0 from
`package/rust-toolchain.toml`. The kit toolchain is Rust 1.89.0 because the
Zenoh crate's dependencies need a newer compiler than the plant package.

## Zenoh

`dynamics-zenoh` is a separate program. It does not replace plugin install.

```bash
cargo run --manifest-path crates/zenoh/Cargo.toml
cargo run --manifest-path crates/zenoh/Cargo.toml -- --key vendor/cornellev/dynamics/bicycle --config zenoh.json5
```

The default queryable is `vendor/cornellev/dynamics/bicycle`. The process keeps
no vehicle state. Each query payload is one 136-byte little-endian request:

- `u32` version `1`
- `u32` step count (`1` for the next state only, at most `64`)
- `f64` dt, wheelbase, and maxSteeringAngle
- 13 `f64` values in `cev-sim.plant-state@1` order

The reply is version, an `i32` code, a count, and that many states. A count
greater than one holds the acceleration and steering from the request and
integrates them for that many steps of `dt`. `0` is success. Negative codes
reject the frame; a positive code comes from the plant.
