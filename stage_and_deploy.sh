#!/bin/bash

# Terminate execution immediately if any internal command pipeline throws an error
set -e

# Validate that the user provided a commit message argument
if [ -z "$1" ]; then
    echo "ERROR: You must provide a descriptive commit message."
    echo "Usage: ./stage_and_deploy.sh \"your meaningful commit message here\""
    exit 1
fi

# Store the command-line argument text string
COMMIT_MSG="$1"

echo "=== Starting MIXR-1 Automated Deployment Pipeline ==="

# Step 1: Track and stage modified/new project blueprints
echo "-> Staging modified code files and environmental assets..."
git add .

# Step 2: Encapsulate snapshots locally with user-defined message strings
echo "-> Creating secure local repository checkpoint..."
git commit -m "$COMMIT_MSG"

# Step 3: Stream updates securely upstream to GitHub servers
echo "-> Synchronizing data repository upward with GitHub remote master..."
git push origin main

echo "=== Deployment Successfully Completed Cleanly ==="
