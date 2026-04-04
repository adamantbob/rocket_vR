use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script in the build directory
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // Tell Cargo where to find the linker script
    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");
}
