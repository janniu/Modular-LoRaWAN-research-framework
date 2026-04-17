#!/usr/bin/env bash
set -euo pipefail

SCRIPT="src/ioj4.pl"
RESULTS_DIR="results"
mkdir -p "$RESULTS_DIR"

# -------------------------------
# FAST TEST CONFIG (for reviewers)
# -------------------------------
SEEDS=(42)
ITERS=10
SCENARIOS=("Industrial")
GATEWAYS=(2)
DEVICES=(150)
declare -A INTERF_MAPPING=( ["high"]="0.5" )
TRAFFIC=("periodic")
CONFIRMED=("0.3")
WEIGHTS=("Balanced")

ALGS="CoATI,Improved-CoATI,Binary-CoATI,GA,PSO,Alg-HR,Alg-LB,Alg-LBHR"

calc_density() { awk -v ed="$1" 'BEGIN { printf "%.2f", ed/100.0 }'; }

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

                tag="TEST_${scen_tag}_GW${gw}_ED${ed}_W_${wtag}_S${seed}"

                echo "perl \"$SCRIPT\" \
--scenario \"$scen\" \
--gw_count \"$gw\" \
--density \"$density\" \
--interf \"$interf\" \
--traffic \"$tr\" \
--confirmed \"$conf\" \
--weight_config \"$w\" \
--alg \"$ALGS\" \
--iters \"$ITERS\" \
--seed \"$seed\" \
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

# -------------------------------
# Run (LOW parallelism for safety)
# -------------------------------
if command -v parallel >/dev/null 2>&1; then
  parallel -j 2 --bar < "$JOBFILE"
else
  xargs -I{} -P 2 bash -lc "{}" < "$JOBFILE"
fi

# -------------------------------
# Consolidate results
# -------------------------------
CONSOL="${RESULTS_DIR}/consolidated_summary.csv"

echo "Algorithm,Metric,Mean,StdDev,CI95_Low,CI95_High,Environment,Devices,Gateways,ED_per_GW,DR_Min,DR_Max,InterferenceLevel,TrafficType,ConfirmedRatio,WeightConfig,Seed,Iters,Config_Tag" > "$CONSOL"

for f in "$RESULTS_DIR"/*_summary.csv; do
  tag=$(basename "$f" | sed 's/_summary\.csv$//')
  tail -n +2 "$f" | awk -v tag="$tag" '{print $0 "," tag}' >> "$CONSOL"
done

echo "Consolidated summary -> $CONSOL"