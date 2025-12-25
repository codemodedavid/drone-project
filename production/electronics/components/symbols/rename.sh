#!/bin/bash

for file in *.kicad_sym; do
    # extract the second line
    second_line=$(sed -n '2p' "$file")

    # extract the quoted string
    newname=$(echo "$second_line" | sed -n 's/.*"\(.*\)".*/\1/p')

    if [ -n "$newname" ]; then
        mv "$file" "${newname}.kicad_sym"
        echo "$file → ${newname}.kicad_sym"
    else
        echo "Skipping $file (no quoted name found)"
    fi
done
