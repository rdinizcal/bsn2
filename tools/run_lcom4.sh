#!/usr/bin/env bash
# Batch existing analyzers only; no metric is implemented here.
set -euo pipefail
cd "$(dirname "$(realpath "$0")")/.."
output="reports/lcom4"
mkdir -p "$output"
rg --files \
  adaptation/adaptation central_hub/central_hub \
  patient/patient sensor/sensor \
  shared_components/shared_components system_monitor/system_monitor \
  -g '*.py' -g '!**/test_components/**' -g '!**/*_tests.py' \
  -g '!**/test_*.py' | LC_ALL=C sort > "$output/files.txt"
mapfile -t files < "$output/files.txt"
.venv-metrics/bin/pyscn analyze --select lcom --json \
  --output "$output/pyscn.json" "${files[@]}" \
  > "$output/pyscn.log" 2> "$output/pyscn.stderr"
.venv-metrics/bin/pyscn analyze --select lcom --html --no-open \
  --output "$output/pyscn-native.html" "${files[@]}" \
  > "$output/pyscn-native.log" 2>&1
PYTHONHASHSEED=0 .venv-metrics/bin/lcom "${files[@]}" \
  > "$output/potfur.txt" 2> "$output/lcom.stderr"
.venv-metrics/bin/pyscn analyze --select lcom --json \
  --output "$output/validation/pyscn.json" "$output/validation/cases.py" \
  > "$output/validation/pyscn.stdout" 2> "$output/validation/pyscn.stderr"
PYTHONHASHSEED=0 .venv-metrics/bin/lcom "$output/validation/cases.py" \
  > "$output/validation/potfur.stdout" 2> "$output/validation/potfur.stderr"
python3 tools/summarize_lcom4.py
printf 'Resultados: %s/report.md e %s/classes.csv\n' "$output" "$output"
