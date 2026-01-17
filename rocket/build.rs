//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

fn main() {
    let target = std::env::var("TARGET").unwrap_or_default();
    let has_rp2040 = std::env::var("CARGO_FEATURE_RP2040").is_ok();
    let has_rp2350 = std::env::var("CARGO_FEATURE_RP2350").is_ok();

    match (target.as_str(), has_rp2040, has_rp2350) {
        ("thumbv6m-none-eabi", true, false) => {}
        ("thumbv8m.main-none-eabihf", false, true) => {}
        _ => {
            let error_msg = format!(
                "\n\x1b[1;31merror\x1b[0m: No chip or target selected!\n\n\
                Please use one of the aliases you defined:\n\
                - For Pico (RP2040):   \x1b[1;32mcargo run-pico\x1b[0m   OR  \x1b[1;32mcargo build-pico\x1b[0m\n\
                - For Pico 2 (RP2350): \x1b[1;32mcargo run-pico2\x1b[0m  OR  \x1b[1;32mcargo build-pico2\x1b[0m\n\
                \n\x1b[1;34mNote\x1b[0m: The current target is '{}' and features are: rp2040={}, rp2350={}\n",
                target, has_rp2040, has_rp2350
            );
            panic!("{}", error_msg);
        }
    }

    // By default, Cargo will re-run a build script whenever any file in the
    // project changes. By specifying `memory.x` here, we ensure the build
    // script is only re-run when `memory.x` itself changes.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");
    println!("cargo:rustc-link-arg=--nmagic");
}
