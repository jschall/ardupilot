#!/bin/sh

THIS_SCRIPT_REL_PATH="Tools/gittools/submodule-sync.sh"
if [ -f "$THIS_SCRIPT_REL_PATH" ]; then
    # we're in the ardupilot root directory
    echo "---------------------------------"
    echo "Deleting ardupilot/build"
    rm build -rf

    LIBCANARD_REL_PATH="modules/libcanard"
    if [ -d "$LIBCANARD_REL_PATH" ]; then
        echo "---------------------------------"
        echo "Deleting $LIBCANARD_REL_PATH"
        rm $LIBCANARD_REL_PATH -rf
        # this sleep is to give a chance for the user to see that we did this.. just in case we care
        sleep 2
    fi

    UAVCAN_REL_PATH="modules/uavcan"
    if [ -d "$UAVCAN_REL_PATH" ]; then
        echo "---------------------------------"
        echo "Deleting $UAVCAN_REL_PATH"
        rm $UAVCAN_REL_PATH -rf
        # this sleep is to give a chance for the user to see that we did this.. just in case we care
        sleep 2
    fi
    echo "---------------------------------"
fi

# this copes with moving origin remote to a new git organisation
# we run it 3 times due to the poor handling of recursion
git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive
