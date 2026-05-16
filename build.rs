// build.rs - Build script for compiling C++ code

fn main() {
    println!("cargo:rerun-if-changed=cpp_shim.cpp");

    // Compile the C++ code
    let mut build = cc::Build::new();
    build
        .cpp(true)  // Enable C++ support
        .include("..")  // Include the parent directory for headers
        .warnings(false);  // Disable warnings as errors for now

    build.file("cpp_shim.cpp");

    // Add every generated algorithm source file from the parent directory.
    // Keeping this dynamic avoids silently missing helper files when the
    // MATLAB Coder output changes.
    for entry in std::fs::read_dir("..").expect("read parent source directory") {
        let path = entry.expect("read source entry").path();
        if path.extension().and_then(|ext| ext.to_str()) == Some("cpp") {
            println!("cargo:rerun-if-changed={}", path.display());
            build.file(path);
        }
    }

    // Compile
    build.compile("covermapobsplan_cpp");

    // On Windows with MSVC, we don't need to link stdc++
    // On other platforms (Linux, macOS), we need to link it
    #[cfg(not(target_env = "msvc"))]
    {
        println!("cargo:rustc-link-lib=stdc++");
    }
}
