---
description: Publish Bucksaw on Wasmer
---

To publish the Wasm version on Wasmer, follow these steps:

1. **Install Prerequisites**
   Ensure you have Rust, `trunk`, and `wasmer` installed.
   ```sh
   # Install trunk (Wasm bundler)
   cargo install trunk
   
   # Add Wasm target
   rustup target add wasm32-unknown-unknown
   
   # Install Wasmer CLI (if not installed)
   curl https://get.wasmer.io -sSfL | sh
   ```

2. **Login to Wasmer**
   ```sh
   wasmer login
   ```

3. **Build data-trunk**
   Clean previous builds and build for release.
   ```sh
   trunk clean
   trunk build --release
   ```
   This will generate the static site in the `dist` directory.

4. **Deploy to Wasmer**
   Deploy the contents of `dist` using the `wasmer.toml` configuration.
   ```sh
   wasmer deploy
   ```
   Follow the interactive prompts to create an app if needed.
   
   > **Note**: If you want to customize the package name, namespace, or app aliases, edit `wasmer.toml` before deploying.
