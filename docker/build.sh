#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/../"

# https://github.com/docker/buildx/issues/484
export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

if [ ! -d "src" ]
then
    mkdir src
    vcs import src < autoware.repos
fi

docker buildx bake --progress=plain -f "$SCRIPT_DIR/autoware-universe/docker-bake.hcl" \
    --set "*.context=$WORKSPACE_ROOT" \
    --set "*.ssh=default" \
    --set "devel.tags=ghcr.io/autowarefoundation/autoware-universe:latest" \
    --set "prebuilt.tags=ghcr.io/autowarefoundation/autoware-universe:latest-prebuilt"
