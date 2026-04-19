#!/bin/bash

usage() { echo "Usage: $0 -b <major|minor|patch>"; }

YAML_CONFIG="../boards_config.yaml"
FW_KEY="FIRMWARE_VERSION"

if [ $# != 2 ]; then
    echo "error: Incorrect arguments"
    usage
    exit 1
fi

if [ $1 != "-b" ]; then
    usage
    exit 2
fi

CURRENT_FW_VERSION=$(py yaml_parser.py -f $YAML_CONFIG --fw_version)
CURRENT_FW_VERSION="${CURRENT_FW_VERSION#v}"
IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT_FW_VERSION"
echo "Current Firmware Version: v$CURRENT_FW_VERSION"

WHICH_TO_BUMP="${2,,}"

case "$WHICH_TO_BUMP" in
    major)
    MAJOR=$((MAJOR+1))
    MINOR=0
    PATCH=0
    ;;
    minor)
    MINOR=$((MINOR+1))
    PATCH=0
    ;;
    patch)
    PATCH=$((PATCH+1))
    ;;
    *)
    usage
    exit 3
    ;;
esac

NEW_FW_VERSION="v"$MAJOR.$MINOR.$PATCH
echo "    New Firmware Version: $NEW_FW_VERSION"

if [[ -n $(git status --porcelain) ]]; then
    echo "error: There are uncommitted changes, please commit and push first."
    exit 1
fi

if [[ -n $(git log origin/$(git branch --show-current)..HEAD) ]]; then
    echo "error: There are unpushed commits, please push first."
    exit 1
fi

read -r -p "Tag and push $NEW_FW_VERSION? [y/N] " confirm
if [[ "${confirm,,}" == "y" ]]; then
    sed -i "s/^\($FW_KEY:\s*\)v[0-9]*\.[0-9]*\.[0-9]*/\1$NEW_FW_VERSION/" $YAML_CONFIG
    git add $YAML_CONFIG
    git commit -m "Release $NEW_FW_VERSION"
    git push origin
    git tag -a "$NEW_FW_VERSION" -m "Release $NEW_FW_VERSION"
    git push origin "$NEW_FW_VERSION"
fi
