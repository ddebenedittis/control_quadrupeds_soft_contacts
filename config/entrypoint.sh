#!/bin/bash
set -e

# Setup environment
source $HOME/.bashrc

# Change owner of file
chown $USER /home/.bash_history

# Start in home directory
