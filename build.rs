use protobuf_codegen_pure::Customize;
use protoc_rust;
use std::fs;
use std::path::Path;

fn main() {
    let out_dir = "./src/communication";

    let generated_with_native_dir = format!("{}/protoduck_generated", out_dir);

    if Path::new(&generated_with_native_dir).exists() {
        fs::remove_dir_all(&generated_with_native_dir).unwrap();
    }

    fs::create_dir(&generated_with_native_dir).unwrap();

    println!("{}", &generated_with_native_dir);
    protoc_rust::Codegen::new()
        .customize(Customize {
            gen_mod_rs: Some(true),
            ..Default::default()
        })
        .out_dir(generated_with_native_dir)
        .input("src/communication/protoduck/messages.proto")
        .include("src/communication/protoduck")
        .run()
        .expect("protoc");
}
