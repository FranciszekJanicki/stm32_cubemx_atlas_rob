#!/usr/bin/bash

for entry in *; do
	if [ -d $entry ]; then
		cd $entry
			cat CMakeLists.txt
		cd ..
	fi
done
