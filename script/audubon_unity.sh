#!/bin/bash

echo "Audubon Unity Script"

# download the latest audubon unity package
roscd audubon_unity
echo "Running in $(pwd)"
git clone https://github.com/SmitRajguru/UB_CSE468_568_audubon_unity.git
cp -R UB_CSE468_568_audubon_unity/* ./
rm -rf UB_CSE468_568_audubon_unity
chmod +x script/*
chmod +x src/*