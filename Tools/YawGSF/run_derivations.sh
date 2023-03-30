#!/bin/sh

mkdir -p generated

python derivations/heading_sigma.py > generated/heading_sigma.cpp
python derivations/init.py > generated/init.cpp
python derivations/zero_rot_err.py > generated/zero_rot_err.cpp
python derivations/predict.py predict > generated/predict.cpp
python derivations/predict.py pnoise > generated/pnoise.cpp
python derivations/quaternion_change.py > generated/quaternion_change.cpp
