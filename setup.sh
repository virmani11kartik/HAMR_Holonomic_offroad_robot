#!/usr/bin/env bash
set -e

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Submodules initialized successfully."
