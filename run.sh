#!/bin/bash

echo "Running simulations..."
bash scripts/run_all_tests_parallel.sh

echo "Generating figures and tables..."
python analysis/make_figures_and_tables.py

echo "Done."