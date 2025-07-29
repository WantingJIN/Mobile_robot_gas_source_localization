#!/bin/bash


trap ctrl_c INT

function ctrl_c() {
  echo -e "\n🛑 Interrupción detectada. Terminando proceso..."
  pkill -P $$  # Mata procesos hijos del script actual
  exit 1
}

# === Rutas base ===
BASE_CONFIG="config_baseline.txt"
CONFIG_FILE="../webots/data/config.txt"
RESULTS_BASE="../Results/combinations_map9"
SIM_SCRIPT="./test_maps_evaluation.sh"  # Tu script de simulación

# === Subconjunto de valores específicos ===
declare -a strategies=("VORONOI" "LLOYD_D" "LLOYD_P" "REGION_GROWING_D" "REGION_GROWING_P")  # X = 1-5
declare -a seeds=("MAX_CELL" "ROBOT_CELL")                                                  # Y = 1-2
planner="SUM_DIJKSTRA_FIX"                                                                  # Z = 3 (fijo)
declare -a gas_features=("ABSOLUTE_VALUE" "RELATIVE_VALUE")                                 # K = 1-2
declare -a score_sigmas=("0.1" "0.9")                                                        # W = 6 ó 7

# Crear carpeta de resultados
mkdir -p "$RESULTS_BASE"


total=$(( ${#strategies[@]} * ${#seeds[@]} * ${#planner[@]} * ${#gas_features[@]} * ${#score_sigmas[@]}))
current=0

# === Contar combinaciones ya completadas (carpetas con >=25 subcarpetas) ===
current=0
for i in "${!strategies[@]}"; do
  for j in "${!seeds[@]}"; do
    for l in "${!gas_features[@]}"; do
      for m in "${!score_sigmas[@]}"; do

        if [[ "${score_sigmas[$m]}" == "0.1" ]]; then
          w="6"
        else
          w="7"
        fi

        id="$((i+1))$((j+1))${w}$((l+1))"
        result_dir="$RESULTS_BASE/$id"

        if [ -d "$result_dir" ]; then
            num_subdirs=$(find "$result_dir" -mindepth 1 -maxdepth 1 -type d | wc -l)
            if [ "$num_subdirs" -ge 25 ]; then
                ((current++))
            fi
        fi

      done
    done
  done
done


# Solo para planner SUM_DIJKSTRA_FIX (Z=3)
z=3

for i in "${!strategies[@]}"; do   # X
  for j in "${!seeds[@]}"; do      # Y
    for l in "${!gas_features[@]}"; do  # K
      for m in "${!score_sigmas[@]}"; do  # scoreSigma

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

        # Generar ID tipo XYWK (Z=6 o 7)
        id="$((i+1))$((j+1))${w}$((l+1))"
        result_dir="$RESULTS_BASE/$id"

        if [ -d "$result_dir" ]; then
            num_subdirs=$(find "$result_dir" -mindepth 1 -maxdepth 1 -type d | wc -l)
            if [ "$num_subdirs" -ge 25 ]; then
                echo "⏩ Combinación $id ya completada (tiene $num_subdirs subcarpetas). Saltando..."
                continue
            fi
        fi

        ((current++))
        progress_percent=$(awk "BEGIN { printf \"%.1f\", ($current/$total)*100 }")
        remaining=$((total - current))
        estimated_seconds=$((remaining * 30 * 20))  # 30 simulaciones × 20s
        h=$((estimated_seconds / 3600))
        m=$(((estimated_seconds % 3600) / 60))
        s=$((estimated_seconds % 60))
        eta=$(printf "%02d:%02d:%02d" $h $m $s)

        echo "🔄 Combinación $current / $total ($progress_percent%) ⏳ Tiempo estimado restante: $eta"

        echo "▶️ Ejecutando combinación $id: $strategy | $seed | $planner | $gasFeature | scoreSigma=$scoreSigma"

        # 1. Copiar config base
        cp "$BASE_CONFIG" "$CONFIG_FILE"

        # 2. Reemplazar en config.txt
        sed -i "s/^mapCoverageStrategy = .*/mapCoverageStrategy = $strategy/" "$CONFIG_FILE"
        sed -i "s/^mapCoverageSeed = .*/mapCoverageSeed = $seed/" "$CONFIG_FILE"
        sed -i "s/^planningStrategy = .*/planningStrategy = $planner/" "$CONFIG_FILE"
        sed -i "s/^gasFeature = .*/gasFeature = $gasFeature/" "$CONFIG_FILE"
        sed -i "s/^scoreSigma = .*/scoreSigma = $scoreSigma/" "$CONFIG_FILE"

        # 3. Crear carpeta de resultados
        mkdir -p "$result_dir"

        # 4. Ejecutar simulación
        echo "  🚀 Lanzando simulaciones..."
        bash "$SIM_SCRIPT"

        # 5. Mover resultados
        mv ../Results/current/* "$result_dir"/ 2>/dev/null

        echo "✅ Resultados guardados en $result_dir"
        echo
        # 6. Ejecutar script para mover carpetas numéricas
        #./move_numeric_dirs.sh

      done
    done
  done
done

echo "🎉 Combinaciones con scoreSigma añadidas correctamente solo para planner SUM_DIJKSTRA_FIX."
