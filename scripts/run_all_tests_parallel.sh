#!/usr/bin/env bash
set -euo pipefail

SCRIPT="ioj4.pl"
RESULTS_DIR="results"
mkdir -p "$RESULTS_DIR"

SEEDS=(42 123 456 789 2025 777 1001 1003 1005 1013)
ITERS=30
SCENARIOS=("Industrial" "Suburban" "Rural-Agricultural")
GATEWAYS=(1 2 4 8)
DEVICES=(150 250 400 800)
declare -A INTERF_MAPPING=( ["high"]="0.5" ["very_high"]="0.7" )
TRAFFIC=("periodic" "bursty")
CONFIRMED=("0.3" "0.7" "1.0")
WEIGHTS=("Favor_RSSI" "Favor_Load_Balancing" "Favor_SNR" "Favor_Interference_Avoidance" "Balanced")
ALGS="CoATI,Improved-CoATI,Binary-CoATI,GA,PSO,Alg-HR,Alg-LB,Alg-LBHR"

calc_density() { awk -v ed="$1" 'BEGIN { printf "%.2f", ed/100.0 }'; }

# Create job list
JOBFILE="jobs.txt"
: > "$JOBFILE"

for scen in "${SCENARIOS[@]}"; do
  for gw in "${GATEWAYS[@]}"; do
    for ed in "${DEVICES[@]}"; do
      for inf_key in "${!INTERF_MAPPING[@]}"; do
        for tr in "${TRAFFIC[@]}"; do
          for conf in "${CONFIRMED[@]}"; do
            for w in "${WEIGHTS[@]}"; do
              for seed in "${SEEDS[@]}"; do
                density=$(calc_density "$ed")
                interf="${INTERF_MAPPING[$inf_key]}"
                scen_tag=$(echo "$scen" | tr ' ' '_' | tr -cd '[:alnum:]_-')
                wtag=$(echo "$w" | tr ' ' '_' | tr -cd '[:alnum:]_-')
                tag="SC_${scen_tag}_GW${gw}_ED${ed}_INF${inf_key}_TR${tr}_CF${conf}_W_${wtag}_S${seed}"
                echo "perl \"$SCRIPT\" --scenario \"$scen\" --gw_count \"$gw\" --density \"$density\" --interf \"$interf\" --traffic \"$tr\" --confirmed \"$conf\" --weight_config \"$w\" --alg \"$ALGS\" --iters \"$ITERS\" --seed \"$seed\" \
&& mv algorithm_results.csv \"$RESULTS_DIR/${tag}_algo.csv\" \
&& mv device_results.csv \"$RESULTS_DIR/${tag}_dev.csv\" \
&& mv summary_statistics.csv \"$RESULTS_DIR/${tag}_summary.csv\"" >> "$JOBFILE"
              done
            done
          done
        done
      done
    done
  done
done

# Run jobs in parallel: adjust -j to your cores / cluster allocation
if command -v parallel >/dev/null 2>&1; then
  parallel -j 16 --bar < "$JOBFILE"
else
  # fallback (less features): 8 concurrent jobs
  xargs -I{} -P 8 bash -lc "{}" < "$JOBFILE"
fi

# Consolidate summaries
CONSOL="${RESULTS_DIR}/consolidated_summary.csv"
echo "Algorithm,Metric,Mean,StdDev,CI95_Low,CI95_High,Environment,Devices,Gateways,ED_per_GW,DR_Min,DR_Max,InterferenceLevel,TrafficType,ConfirmedRatio,WeightConfig,Seed,Iters,Config_Tag" > "$CONSOL"
for f in "$RESULTS_DIR"/*_summary.csv; do
  tag=$(basename "$f" | sed 's/_summary\.csv$//')
  tail -n +2 "$f" | awk -v tag="$tag" '{print $0 "," tag}' >> "$CONSOL"
done
echo "Consolidated summary -> $CONSOL"

