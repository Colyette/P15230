#timestamp log
#_now=$(date +"%m_%d_%Y")
_now=$(date +%Y-%m-%d_%H-%M-%S)
_file="logs/log_$_now.txt"
mkdir -p logs
echo $_now > $_file
#start program, send output to a log
./test >> $_file 2>&1
