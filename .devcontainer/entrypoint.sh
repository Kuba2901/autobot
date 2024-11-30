#!/bin/bash

rosdep update
rosdep install --from-paths /carlike_bot/src --ignore-src -r -y
echo "ROSDEP UPDATE COMPLETE"