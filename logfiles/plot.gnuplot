
set datafile separator whitespace
set title "Moisture vs Threshold (Sen 127 & 128)"
set xlabel "Index"
set ylabel "Mo / Threshold Value"
set grid
set key left top

plot "parsed_data.txt" using 1:4 with linespoints title "Mo127", \
     "parsed_data.txt" using 1:3 with lines dashtype 2 title "Thr127", \
     "parsed_data.txt" using 1:6 with linespoints title "Mo128", \
     "parsed_data.txt" using 1:5 with lines dashtype 2 title "Thr128"
