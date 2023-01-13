# get unix system time (ms)
TIME_MS=$(date +%s%3N)

# convert ms time to human readable
TIME=$(date -d @$((TIME_MS/1000)) +%Y-%m-%d-%H-%M-%S)