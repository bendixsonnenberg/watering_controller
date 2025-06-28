
import sys
import re

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} input_file")
    sys.exit(1)

input_file = sys.argv[1]
output_file = "parsed_data.txt"

index = 0
line_count = 0

with open(input_file, "r") as fin, open(output_file, "w") as fout:
    fout.write("Index Time Thr127 Mo127 Thr128 Mo128\n")

    for line in fin:
        line_count += 1
        if line_count % 1000 == 0:
            print(f"Processed {line_count} lines...")

        time_match = re.search(r'Time:\s*(\d+)', line)
        if not time_match:
            continue
        time = time_match.group(1)

        readings = re.findall(r'Sen:\s*(\d+),\s*Thr:\s*(\d+),\s*Mo:\s*(\d+)', line)

        thr127 = mo127 = None
        thr128 = mo128 = None

        for sen, thr, mo in readings:
            if sen == "127":
                thr127 = thr
                mo127 = mo
            elif sen == "128":
                thr128 = thr
                mo128 = mo

        if thr127 is not None and mo127 is not None and thr128 is not None and mo128 is not None:
            fout.write(f"{index} {time} {thr127} {mo127} {thr128} {mo128}\n")
            index += 1

print(f"Finished processing {line_count} lines, wrote {index} valid entries to {output_file}.")
