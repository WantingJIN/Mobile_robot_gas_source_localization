#!/bin/bash

RESULTS_DIR="../Results"
COMBINATIONS_DIR="$RESULTS_DIR/combinations_map9"
moved_any=false

numeric_dirs=()

# Buscar carpetas con nombre numérico dentro de RESULTS_DIR
for dir in "$RESULTS_DIR"/*; do
  if [[ -d "$dir" ]]; then
    name=$(basename "$dir")
    if [[ "$name" =~ ^[0-9]+$ ]]; then
      numeric_dirs+=("$name")
    fi
  fi
done

echo "🔍 Carpetas numéricas encontradas: ${numeric_dirs[*]}"

if [ "${#numeric_dirs[@]}" -ge 1 ]; then
  # Buscar carpeta más reciente en combinations/
  latest_comb_dir=$(ls -td "$COMBINATIONS_DIR"/*/ 2>/dev/null | head -n 1)

  if [ -n "$latest_comb_dir" ]; then
    echo "📦 Carpeta de destino más reciente: $latest_comb_dir"

    for dir in "${numeric_dirs[@]}"; do
      src="$RESULTS_DIR/$dir"
      echo "➡️ Moviendo $src a $latest_comb_dir"
      mv "$src" "$latest_comb_dir"
      moved_any=true
    done
  else
    echo "❌ No hay carpeta en $COMBINATIONS_DIR"
  fi
else
  echo "⚠️ No hay suficientes carpetas numéricas para mover (encontradas: ${#numeric_dirs[@]})"
fi

# Terminar si movió algo
if $moved_any; then
  echo "✅ Carpetas movidas. Script terminado."
  exit 0
else
  echo "ℹ️ No se movió nada. Script terminado."
  exit 0
fi
