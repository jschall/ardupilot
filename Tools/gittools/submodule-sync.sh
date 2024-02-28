#!/bin/sh

THIS_SCRIPT_REL_PATH="Tools/gittools/submodule-sync.sh"
if [ -f "$THIS_SCRIPT_REL_PATH" ]; then
    # we're in the ardupilot root directory
    echo "---------------------------------"
    echo "Deleting ardupilot/build"
    rm build -rf

    echo "Deleting ardupilot/modules"
    rm modules -rf
    echo "---------------------------------"
    echo
fi

# this copes with moving origin remote to a new git organisation
# we run it 3 times due to the poor handling of recursion
git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive


./waf configure --board CubeOrange-KHA
echo "---------------------------------"
echo "Configured CubeOrange-KHA"
echo "---------------------------------"
