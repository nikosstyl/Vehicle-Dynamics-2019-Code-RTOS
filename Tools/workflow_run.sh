#!/bin/bash

TAG_NAME=${GITHUB_REF_NAME}

mkdir -p release

BOARD_CONFIGS=$(python3 ./Tools/yaml_parser.py -f boards_config.yaml -p)

for board in $BOARD_CONFIGS; do
    BUILD_DIR=build_${board}
    YELLOW='\033[1;33m'
    NC='\033[0m' # No Color
    echo -e "${YELLOW}Building variant ${board}${NC}"

    cmake -B $BUILD_DIR \
        -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
        -DBOARD=${board} \
        -G Ninja

    cmake --build $BUILD_DIR

    # Find ELF
    ELF_FILE=$(find $BUILD_DIR -maxdepth 1 -type f -name "*.elf")
    ELF_FILE_NO_EXT="${ELF_FILE%.*}"
    NEW_ELF="release/$(basename $ELF_FILE_NO_EXT)-${board}-$TAG_NAME.elf"
    cp "$ELF_FILE" "$NEW_ELF"
    # echo "Packaged ELF: $NEW_ELF"
done
