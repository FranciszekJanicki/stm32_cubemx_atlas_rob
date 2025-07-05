#!/bin/bash
set -e

function update_component {
    echo "Updating component: $1"
    git submodule update --remote "$1"
    echo "Done."
}

git submodule init

for dir in components/*; do
    if [ -d "$dir/.git" ] || [ -f "$dir/.git" ]; then
        update_component "$dir"
    fi
done

