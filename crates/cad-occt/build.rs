use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root")
        .to_path_buf();

    let occt_root = env::var("OCCT_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|_| workspace_root.join("build").join("occt-install"));

    let mut include_dirs = Vec::new();
    let unix_inc = occt_root.join("include").join("opencascade");
    let win_inc = occt_root.join("inc");
    let build_inc = workspace_root.join("build").join("occt-build").join("inc");

    for dir in [unix_inc, win_inc, build_inc] {
        if dir.exists() {
            include_dirs.push(dir);
        }
    }

    if include_dirs.is_empty() {
        panic!(
            "OCCT include directory not found under {}. Run `py scripts/build_occt.py` first.",
            occt_root.display()
        );
    }

    let mut build = cxx_build::bridge("src/lib.rs");
    build
        .file("src/occt_bridge.cpp")
        .std("c++17")
        .include(manifest_dir.parent().expect("crates dir"));

    for inc in &include_dirs {
        build.include(inc);
    }

    if cfg!(target_env = "msvc") {
        build.flag("/EHsc");
        build.flag("/bigobj");
        // Ensure dynamic release CRT (/MD) matching OCCT Release static libs and Rust default
        build.static_crt(false);
    }

    build.compile("ferrocad_occt_bridge");

    // Link OCCT libraries
    let mut lib_dirs = Vec::new();
    let unix_lib = occt_root.join("lib");
    let build_lib_release = workspace_root
        .join("build")
        .join("occt-build")
        .join("win64")
        .join("vc14")
        .join("lib");

    for dir in [unix_lib, build_lib_release] {
        if dir.exists() {
            lib_dirs.push(dir);
        }
    }

    if lib_dirs.is_empty() {
        panic!(
            "OCCT lib directory not found under {}. Run `py scripts/build_occt.py` first.",
            occt_root.display()
        );
    }

    // Link OCCT libraries in strict topological dependency order (base -> derived)
    // so that MSVC .CRT$XCU static constructors initialize TKernel first and
    // LIFO atexit destructors destroy TKDESTEP before TKernel.
    let ordered_toolkits = [
        "TKernel",
        "TKMath",
        "TKG2d",
        "TKG3d",
        "TKGeomBase",
        "TKBRep",
        "TKGeomAlgo",
        "TKTopAlgo",
        "TKPrim",
        "TKBO",
        "TKShHealing",
        "TKBool",
        "TKHLR",
        "TKFillet",
        "TKOffset",
        "TKMesh",
        "TKService",
        "TKV3d",
        "TKCDF",
        "TKLCAF",
        "TKCAF",
        "TKVCAF",
        "TKBinL",
        "TKBin",
        "TKXCAF",
        "TKBinXCAF",
        "TKXSBase",
        "TKDE",
        "TKDESTEP",
    ];

    for lib_dir in &lib_dirs {
        println!("cargo:rustc-link-search=native={}", lib_dir.display());
    }

    for tk in &ordered_toolkits {
        println!("cargo:rustc-link-lib=static={tk}");
    }

    if cfg!(target_os = "windows") {
        for sys_lib in [
            "user32", "advapi32", "gdi32", "ws2_32", "winmm", "shell32", "ole32", "oleaut32",
        ] {
            println!("cargo:rustc-link-lib=dylib={sys_lib}");
        }
    }

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/occt_bridge.cpp");
    println!("cargo:rerun-if-changed=include/occt_bridge.h");
    println!("cargo:rerun-if-env-changed=OCCT_ROOT");
}
