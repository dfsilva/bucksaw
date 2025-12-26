//! Build script that loads environment variables from .env file
//! 
//! This allows setting GA4_MEASUREMENT_ID and GA4_API_SECRET in a .env file
//! instead of passing them on the command line.

use std::fs;
use std::path::Path;

fn main() {
    // Tell Cargo to rerun if .env changes
    println!("cargo:rerun-if-changed=.env");
    println!("cargo:rerun-if-changed=.env.local");

    // Try to load .env.local first (for local overrides), then .env
    let env_files = [".env.local", ".env"];
    
    for env_file in env_files {
        if let Ok(contents) = fs::read_to_string(Path::new(env_file)) {
            for line in contents.lines() {
                let line = line.trim();
                
                // Skip empty lines and comments
                if line.is_empty() || line.starts_with('#') {
                    continue;
                }
                
                // Parse KEY=VALUE format
                if let Some((key, value)) = line.split_once('=') {
                    let key = key.trim();
                    let value = value.trim();
                    
                    // Only set if not already set in environment
                    if std::env::var(key).is_err() {
                        println!("cargo:rustc-env={}={}", key, value);
                    }
                }
            }
            
            // Only load the first .env file found
            break;
        }
    }
}
