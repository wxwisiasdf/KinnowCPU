#!/bin/sh
cd gcc || exit
git pull || exit
git add . || exit
git diff --cached >../gcc-patch.diff || exit
cd .. || exit
