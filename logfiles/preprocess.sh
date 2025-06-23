#!/bin/bash

input_file="$1"
output_file="parsed_data.txt"

# Header
echo "Index Time Thr127 Mo127 Thr128 Mo128" > "$output_file"

index=0

while read -r line; do
    # Extract timestamp
    time=$(echo "$line" | grep -o 'Time: *[0-9]*' | grep -o '[0-9]*')

    # Skip if Time not found
    if [[ -z "$time" ]]; then
        continue
    fi

    if (( index % 1000 == 0 )); then
        echo "Processed $line_count lines..."
    fi    # Extract all sensor blocks
    readings=$(echo "$line" | grep -o 'Sen: *[0-9]*, *Thr: *[0-9]*, *Mo: *[0-9]*')

    # Initialize values
    thr127="" mo127=""
    thr128="" mo128=""

    # Loop over sensor readings
    while read -r sensor; do
        sen=$(echo "$sensor" | grep -o 'Sen: *[0-9]*' | grep -o '[0-9]*')
        thr=$(echo "$sensor" | grep -o 'Thr: *[0-9]*' | grep -o '[0-9]*')
        mo=$(echo "$sensor" | grep -o 'Mo: *[0-9]*' | grep -o '[0-9]*')

        if [[ "$sen" == "127" ]]; then
            thr127=$thr
            mo127=$mo
        elif [[ "$sen" == "128" ]]; then
            thr128=$thr
            mo128=$mo
        fi
    done <<< "$readings"

    # Only write lines that have both sensor readings
    if [[ -n "$thr127" && -n "$mo127" && -n "$thr128" && -n "$mo128" ]]; then
        echo "$index $time $thr127 $mo127 $thr128 $mo128" >> "$output_file"
        index=$((index + 1))
    fi

done < "$input_file"
