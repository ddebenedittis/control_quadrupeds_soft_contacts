#!/bin/bash
set -e

# Setup environment
source $HOME/.bashrc
source /usr/share/gazebo/setup.bash

# Change owner of file
chown $USER /home/.bash_history

# Start in home directory
