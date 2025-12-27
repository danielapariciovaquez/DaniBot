#!/bin/sh
set -e

cd /home/dani/DaniBot
export GIT_TERMINAL_PROMPT=0

git pull

exec /usr/bin/python3 /home/dani/DaniBot/canProgram.py
