#!/usr/bin/env python3
import pandas as pd
import numpy as np
import re
import os
import matplotlib.pyplot as plt

plt.rcParams.update({"figure.dpi": 180, "axes.grid": True})

RES_DIR = "results"
CONS = os.path.join(RES_DIR, "consolidated_summary.csv")
OUT = os.path.join(RES_DIR, "figs")
TAB = os.path.join(RES_DIR, "tables")
os.makedirs(OUT, exist_ok=True)
os.makedirs(TAB, exist_ok=True)

df = pd.read_csv(CONS)

# Parse config tag columns (SC_<scen>_GW<g>_ED<d>_INF<k>_TR<tr>_CF<c>_W_<w>_S<seed>)
def parse_tag(tag):
    out = {}
    m = re.search(r"SC_([^_]+)_GW(\d+)_ED(\d+)_INF([^_]+)_TR([^_]+)_CF([0-9.]+)_W_([^_]+)_S(\d+)", tag)
    if m:
        out["ScenarioTag"] = m.group(1)
        out["GW"] = int(m.group(2))
        out["ED"] = int(m.group(3))
        out["INF"] = m.group(4)
        out["TR"] = m.group(5)
        out["CF"] = float(m.group(6))
        out["Wtag"] = m.group(7)
        out["Seed"] = int(m.group(8))
    return out

cfg = df["Config_Tag"].apply(parse_tag).apply(pd.Series)
df = pd.concat([df, cfg], axis=1)

# --- Per-scenario tables (IoT-J style) ---
key_metrics = ["pdr","eff_pdr","energy","delay","throughput","ack_rate","retx_rate","ul_lat_mean","ul_lat_p95","fairness"]

for scen in sorted(df["Environment"].unique()):
    sub = df[df["Environment"] == scen]
    # take means across seeds/weights/interf/traffic per algorithm & ED/GW if needed
    # Table 1: Algorithm overall means (most common in journals)
    t1 = (sub
          .groupby(["Algorithm","Metric"])["Mean"]
          .mean()
          .unstack("Metric")
          .reindex(columns=key_metrics, fill_value=np.nan))
    t1.to_csv(os.path.join(TAB, f"table_overall_{scen}.csv"))
    
    # Table 2 (optional): break by device count for PDR/Energy
    t2 = (sub[sub["Metric"].isin(["pdr","energy"])]
          .groupby(["ED","Algorithm","Metric"])["Mean"]
          .mean()
          .unstack("Metric")
          .sort_index())
    t2.to_csv(os.path.join(TAB, f"table_by_devices_{scen}.csv"))

# --- Plots: PDR vs #Devices (per scenario) ---
for scen in sorted(df["Environment"].unique()):
    sub = df[(df["Environment"] == scen) & (df["Metric"] == "pdr")]
    # average across seeds/weights/interf/traffic per (ED, Algorithm)
    agg = sub.groupby(["ED","Algorithm"])["Mean"].mean().reset_index()
    pivot = agg.pivot(index="ED", columns="Algorithm", values="Mean").sort_index()
    ax = pivot.plot(marker="o")
    ax.set_title(f"PDR vs Devices — {scen}")
    ax.set_xlabel("End Devices (ED)")
    ax.set_ylabel("PDR (mean)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, f"pdr_vs_devices_{scen}.png"))
    plt.clf()

# --- Plots: Energy vs Gateways (per scenario) ---
for scen in sorted(df["Environment"].unique()):
    sub = df[(df["Environment"] == scen) & (df["Metric"] == "energy")]
    agg = sub.groupby(["GW","Algorithm"])["Mean"].mean().reset_index()
    pivot = agg.pivot(index="GW", columns="Algorithm", values="Mean").sort_index()
    ax = pivot.plot(marker="o")
    ax.set_title(f"Energy vs Gateways — {scen}")
    ax.set_xlabel("Gateways")
    ax.set_ylabel("Energy (mJ, mean)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, f"energy_vs_gateways_{scen}.png"))
    plt.clf()

# --- Plots: Latency (mean & P95) bars per algorithm (per scenario) ---
for scen in sorted(df["Environment"].unique()):
    sub_mean = df[(df["Environment"] == scen) & (df["Metric"] == "ul_lat_mean")]
    sub_p95  = df[(df["Environment"] == scen) & (df["Metric"] == "ul_lat_p95")]
    m = sub_mean.groupby("Algorithm")["Mean"].mean()
    p = sub_p95.groupby("Algorithm")["Mean"].mean()
    lat = pd.DataFrame({"mean": m, "p95": p})
    ax = lat.plot(kind="bar")
    ax.set_title(f"Uplink Latency (Mean & P95) — {scen}")
    ax.set_xlabel("Algorithm")
    ax.set_ylabel("Latency (ms)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, f"latency_bars_{scen}.png"))
    plt.clf()

# --- Where do CoATI variants win? (dense/high interference, low GW) ---
dense_mask = df["ED"] >= 400
high_inf   = df["InterferenceLevel"].isin(["high","very_high"]) | df["INF"].isin(["high","very_high"])
low_gw     = df["GW"].isin([1,2])
focus = df[dense_mask & high_inf & low_gw]

def leaderboard(metric):
    sub = focus[focus["Metric"] == metric]
    agg = sub.groupby(["Environment","GW","ED","Algorithm"])["Mean"].mean().reset_index()
    lead = (agg.sort_values(["Environment","GW","ED","Mean"], ascending=[True,True,True,False])
              .groupby(["Environment","GW","ED"])
              .head(1))
    return lead

winners_pdr = leaderboard("pdr")
winners_eff = leaderboard("eff_pdr")

summary = (winners_pdr
           .merge(winners_eff, on=["Environment","GW","ED"], suffixes=("_pdr","_eff")))
summary.to_csv(os.path.join(TAB, "coati_win_summary_dense_highinf_lowgw.csv"), index=False)

# Quick text dump to help writing:
counts = summary["Algorithm_pdr"].value_counts().rename_axis('Algorithm').reset_index(name='Wins_PDR')
counts2= summary["Algorithm_eff"].value_counts().rename_axis('Algorithm').reset_index(name='Wins_EffPDR')
counts.merge(counts2, on="Algorithm", how="outer").fillna(0).to_csv(os.path.join(TAB,"coati_win_counts.csv"), index=False)

print("Tables in:", TAB)
print("Figures in:", OUT)

