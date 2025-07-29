#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
  echo -e "\n🛑 Interrupción detectada. Terminando proceso..."
  pkill -P $$
  exit 1
}

# === Rutas base ===
BASE_CONFIG="config_baseline.txt"
CONFIG_FILE="../webots/data/config.txt"
RESULTS_BASE="../Results/combinations_map9"
SIM_SCRIPT="./test_maps_evaluation.sh"

# === Valores posibles ===
declare -a strategies=("VORONOI" "LLOYD_D" "LLOYD_P" "REGION_GROWING_D" "REGION_GROWING_P")  # X = 1-5
declare -a seeds=("MAX_CELL" "ROBOT_CELL")                                                  # Y = 1-2
declare -a gas_features=("ABSOLUTE_VALUE" "RELATIVE_VALUE")                                 # K = 1-2
declare -a score_sigmas=("0.1" "0.9")                                                        # W = 6 ó 7
planner="SUM_DIJKSTRA_FIX"                                                                   # Z = fijo

# Crear carpeta de resultados
mkdir -p "$RESULTS_BASE"

# === Total de combinaciones esperadas ===
total=$(( ${#strategies[@]} * ${#seeds[@]} * ${#gas_features[@]} * ${#score_sigmas[@]} ))
current=0

# === Ejecución para combinaciones incompletas (<30) ===
for i in "${!strategies[@]}"; do
  for j in "${!seeds[@]}"; do
    for l in "${!gas_features[@]}"; do
      for m in "${!score_sigmas[@]}"; do

        strategy=${strategies[$i]}      # X
        seed=${seeds[$j]}              # Y
        gasFeature=${gas_features[$l]} # K
        scoreSigma=${score_sigmas[$m]} # W

        # scoreSigma → W (6 o 7)
        if [[ "$scoreSigma" == "0.1" ]]; then
          w="6"
        else
          w="7"
        fi

        id="$((i+1))$((j+1))${w}$((l+1))"
        result_dir="$RESULTS_BASE/$id"

        if [ -d "$result_dir" ]; then
          num_subdirs=$(find "$result_dir" -mindepth 1 -maxdepth 1 -type d | wc -l)
          if [ "$num_subdirs" -lt 30 ]; then
            missing=$((30 - num_subdirs))
            echo "⏩ Combinación $id tiene $num_subdirs subcarpetas. Faltan $missing. Ejecutando..."

            # Actualizar configuración
            cp "$BASE_CONFIG" "$CONFIG_FILE"
            sed -i "s/^mapCoverageStrategy = .*/mapCoverageStrategy = $strategy/" "$CONFIG_FILE"
            sed -i "s/^mapCoverageSeed = .*/mapCoverageSeed = $seed/" "$CONFIG_FILE"
            sed -i "s/^planningStrategy = .*/planningStrategy = $planner/" "$CONFIG_FILE"
            sed -i "s/^gasFeature = .*/gasFeature = $gasFeature/" "$CONFIG_FILE"
            sed -i "s/^scoreSigma = .*/scoreSigma = $scoreSigma/" "$CONFIG_FILE"

            for ((n=1; n<=missing; n++)); do
              bash "$SIM_SCRIPT"
              mv ../Results/current/* "$result_dir"/ 2>/dev/null
            done
          else
            echo "✅ Combinación $id ya completa con $num_subdirs simulaciones."
          fi
        fi

      done
    done
  done
done

echo "🎉 Todas las combinaciones necesarias se han completado (planner = $planner)."
