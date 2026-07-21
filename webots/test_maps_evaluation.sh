#!/bin/bash
now=$(date +"%T")
echo "Start time : $now"

webots=$WEBOTS_HOME/webots
webots_folder="../webots"
source_pos_filename="../webots/data/source_pos.txt"
rr_filename="../webots/data/plugin_parameters/releaseAmount.txt"
config_file="../webots/data/config.yaml"
source_locations_filename="source_locations.txt"
# Source position mode: source_at_boundary, source_in_middle, source_from_file
source_position_mode="source_in_middle"

# ---------------------------------------------------------------------------
# Helper: set a scalar value in the YAML config file.
# Works for any unique key in the file regardless of nesting depth, because
# it matches leading whitespace + key name.
# Usage: set_yaml_value <key> <value>
# ---------------------------------------------------------------------------
set_yaml_value() {
    local key="$1"
    local value="$2"
    sed -i "s/\(^[[:space:]]*${key}:[[:space:]]*\).*/\1${value}/" "$config_file"
}

# ---------------------------------------------------------------------------
# Source position bounds
# ---------------------------------------------------------------------------
nbr_source_pos=10
source_pos_y_min=0.5
source_pos_y_max=3.5

# source_pos_x_min=4
# source_pos_x_max=11.5
source_pos_x_min=10
source_pos_x_max=12



# Return the k-th of n deterministic positions on an X.  The first position is
# the centre; the remaining positions are spread evenly over the four arms,
# with the last position on each used arm reaching a corner of the bounds.
x_source_position() {
	awk -v k="$1" -v n="$2" \
		-v xmin="$source_pos_x_min" -v xmax="$source_pos_x_max" \
		-v ymin="$source_pos_y_min" -v ymax="$source_pos_y_max" '
	BEGIN {
		xmid = (xmin + xmax) / 2
		ymid = (ymin + ymax) / 2
		if (n <= 1 || k <= 1) {
			printf "%.6f %.6f\n", xmid, ymid
			exit
		}

		point = k - 2
		arm = point % 4
		layer = int(point / 4) + 1
		arm_count = int((n - 2 - arm) / 4) + 1
		radius = layer / arm_count
		xsign = (arm < 2) ? 1 : -1
		ysign = (arm == 0 || arm == 3) ? 1 : -1

		x = xmid + xsign * radius * (xmax - xmin) / 2
		y = ymid + ysign * radius * (ymax - ymin) / 2
		printf "%.6f %.6f\n", x, y
	}'
}



random_between() {
    local min="$1"
    local max="$2"
    awk -v min="$min" -v max="$max" -v seed="$RANDOM" \
        'BEGIN { srand(seed); print min + rand() * (max - min) }'
}

# ---------------------------------------------------------------------------
# Delete last unfinished result folder
# ---------------------------------------------------------------------------
result_folder_name="../Results/current/"
if [ -d "$result_folder_name" ]; then
    echo "Folder exists. Deleting..."
    rm -r "$result_folder_name"
    echo "Folder deleted."
fi

# ---------------------------------------------------------------------------
# Parameter sweeps — uncomment the loops you want to sweep over.
# Each set_yaml_value call patches the YAML in-place before launching Webots.
#
# Available sweep parameters (add/remove as needed):
#   bayesian_estimation: likelihood_error, R0, likelihoodSigma, lambda
#   planning:            planningStrategy, scoreFun, scoreSigma, infoFeature
#                        weighting_entropy_ratio (active for SUM_DIJKSTRA_ADP)
#                        cluster_entropy_ratio   (active for BELIEF_CLUSTERING)
#   gas_feature:         gasHitThres (active for FIXED_THRES / FIXED_THRES_NUM)
# ---------------------------------------------------------------------------

# release amount
for r in 53 #10 30 53 80 200
do
    echo $r > $rr_filename

    for m in 10 15 #1 2 3 5 10
    do
        set_yaml_value map_nbr $m
        env_folder="../openfoam/test_maps/map${m}/map${m}_wind0.75"
        source_locations_file="${env_folder}/${source_locations_filename}"

        source_locations=()
        source_locations_count=0
        case "$source_position_mode" in
            source_from_file)
                if [ -f "$source_locations_file" ]; then
                    mapfile -t source_locations < <(grep -Ehv '^\s*(#|$)' "$source_locations_file")
                fi
                source_locations_count=${#source_locations[@]}
                if [ "$source_locations_count" -gt 0 ]; then
                    echo "Use source locations from $source_locations_file"
                else
                    echo "No source locations file (or empty) for map $m, using random source positions"
                fi
                ;;
            source_at_boundary)
                echo "Use random source positions at x boundary: $source_pos_x_max"
                ;;
            source_in_middle)
                echo "Use deterministic X-shaped source positions inside bounds: x=[$source_pos_x_min, $source_pos_x_max], y=[$source_pos_y_min, $source_pos_y_max]"
                ;;
            *)
                echo "Unknown source_position_mode: $source_position_mode" >&2
                echo "Expected: source_at_boundary, source_in_middle, or source_from_file" >&2
                exit 1
                ;;
        esac

        # -- parameter sweep loops (uncomment to activate) ------------------

        # for likelihood_error in 2000 3000
        # do
        #     set_yaml_value likelihood_error $likelihood_error

        # for likelihoodSigma in 0.43 0.45 0.48 3.5
        # do
        #     set_yaml_value likelihoodSigma $likelihoodSigma

        # for R0 in 100 1500
        # do
        #     set_yaml_value R0 $R0

        # for lambda in 0.5 0.7 0.9
        # do
        #     set_yaml_value lambda $lambda

        # for gasHitThres in 100 300 1000
        # do
        #     set_yaml_value gasHitThres $gasHitThres

        # for planningStrategy in BELIEF_CLUSTERING SUM_DIJKSTRA_ADP INFORMATIVE_KLD
        # do
        #     set_yaml_value planningStrategy $planningStrategy

        # for scoreFun in RATIO LOG EXP
        # do
        #     set_yaml_value scoreFun $scoreFun

        # for scoreSigma in 0.4 0.5 0.7
        # do
        #     set_yaml_value scoreSigma $scoreSigma

        # for infoFeature in VARIANCE MEAN PDF
        # do
        #     set_yaml_value infoFeature $infoFeature

        # for weighting_entropy_ratio in 5 10 20
        # do
        #     set_yaml_value weighting_entropy_ratio $weighting_entropy_ratio

        # for cluster_entropy_ratio in 2 4 8
        # do
        #     set_yaml_value cluster_entropy_ratio $cluster_entropy_ratio

        # -------------------------------------------------------------------

        # copy environment data
        cp "$env_folder"/"$m".txt data/plugin_parameters/wind_map.txt
        cp "$env_folder"/wind_map_2D_"$m".csv data/wind_map_2D.csv
        cp "$env_folder"/"$m"_obstacles.txt data/obstacle_list.txt
        cp ../Results/test_maps/map"$m"/test_map"$m".environment data/test_map.environment

        for ((k=1; k<=$nbr_source_pos; k++))
        do
            case "$source_position_mode" in
                source_at_boundary)
                    source_pos_x=$source_pos_x_max
                    source_pos_y=$(random_between "$source_pos_y_min" "$source_pos_y_max")
                    ;;
                source_in_middle)
                    read -r source_pos_x source_pos_y <<< "$(x_source_position "$k" "$nbr_source_pos")"
                    ;;
                source_from_file)
                    if [ "$source_locations_count" -gt 0 ]; then
                        loc_idx=$(( (k - 1) % source_locations_count ))
                        read -r source_pos_x source_pos_y <<< "${source_locations[$loc_idx]}"
                    else
                        source_pos_x=$(random_between "$source_pos_x_min" "$source_pos_x_max")
                        source_pos_y=$(random_between "$source_pos_y_min" "$source_pos_y_max")
                    fi
                    ;;
            esac
            echo $source_pos_x $source_pos_y > $source_pos_filename
            $webots '--minimize' '--mode=fast' $webots_folder'/worlds/WT_test_maps.wbt' &> wb_output.txt
            now=$(date +"%T")
            echo "map $m  k $k end time $now"
        done

        # -- close sweep loops in the same order as opened (uncomment matching done lines) --
        # done  # cluster_entropy_ratio
        # done  # weighting_entropy_ratio
        # done  # infoFeature
        # done  # scoreSigma
        # done  # scoreFun
        # done  # planningStrategy
        # done  # gasHitThres
        # done  # lambda
        # done  # R0
        # done  # likelihoodSigma
        # done  # likelihood_error

    done  # map
done  # release amount

r=53
echo "$r" > $rr_filename

now=$(date +"%T")
echo "End time : $now"
