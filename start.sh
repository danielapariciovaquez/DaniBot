#!/bin/sh
set -e

cd /home/dani/DaniBot
export GIT_TERMINAL_PROMPT=0

git checkout -- start.sh
git pull

#exec /usr/bin/python3 /home/dani/DaniBot/confRobot.py
exec /usr/bin/python3 /home/dani/DaniBot/daniBot.py
