#!/usr/bin/env bash
# Build the single-file page: Rust core → WASM (base64-embedded) + TypeScript UI bundle.
set -euo pipefail
cd "$(dirname "$0")"
( cd ../rust && RUSTC_BOOTSTRAP=1 cargo build --release --lib --target wasm32-unknown-unknown -Zbuild-std=std,panic_abort -q )
../ts/node_modules/.bin/tsc -p .
../ts/node_modules/.bin/esbuild src/main.ts --bundle --format=iife --target=es2020 --minify --outfile=dist/ui.js --log-level=warning
python3 - <<'PY'
import base64
page = open('page.html').read()
wasm = base64.b64encode(open('../rust/target/wasm32-unknown-unknown/release/circuitsim.wasm', 'rb').read()).decode()
ui = open('dist/ui.js').read()
out = page.replace('/*WASM*/', wasm).replace('/*BUNDLE*/', ui)
open('../circuit-autopilot-sim.html', 'w').write(out)
print(f"page {len(out)//1024} KB (wasm {len(wasm)//1024} KB base64, ui {len(ui)//1024} KB)")
PY
