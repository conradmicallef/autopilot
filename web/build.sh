#!/usr/bin/env bash
# Build the single-file page: Rust core → WASM (base64-embedded) + TypeScript UI bundle.
set -euo pipefail
cd "$(dirname "$0")"
( cd ../rust && RUSTC_BOOTSTRAP=1 cargo build --profile wasm --lib --target wasm32-unknown-unknown -Zbuild-std=std,panic_abort -q )
[ -x node_modules/.bin/wasm-opt ] && [ -x node_modules/.bin/esbuild ] || npm i -s
mkdir -p dist
node_modules/.bin/wasm-opt -Oz --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext --enable-mutable-globals \
  ../rust/target/wasm32-unknown-unknown/wasm/circuitsim.wasm -o dist/circuitsim.wasm
node_modules/.bin/tsc -p .
node_modules/.bin/esbuild src/main.ts --bundle --format=iife --target=es2020 --minify --outfile=dist/ui.js --log-level=warning
python3 - <<'PY'
import base64, gzip
page = open('page.html').read()
raw = open('dist/circuitsim.wasm', 'rb').read()
wasm = base64.b64encode(gzip.compress(raw, 9, mtime=0)).decode()   # unpacked in the page with DecompressionStream
ui = open('dist/ui.js').read()
out = page.replace('/*WASM*/', wasm).replace('/*BUNDLE*/', ui)
open('../circuit-autopilot-sim.html', 'w').write(out)
print(f"page {len(out)//1024} KB (wasm {len(raw)//1024} KB, embedded gzip+base64 {len(wasm)//1024} KB; ui {len(ui)//1024} KB)")
PY
