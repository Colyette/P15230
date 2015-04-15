#timestamp log
#_now=$(date +"%m_%d_%Y")
_now=$(date +%Y-%m-%d_%H-%M-%S)
_file="logs/log_$_now.txt"

#change to prog directory
cd /home/pi/c_code/platfromCom/

#make log directory (if not) and create log
mkdir -p logs
echo $_now > $_file

#Assusmes program lmnoCopter is precompiled
#start program, send output to created log
./lmnoCopter >> $_file 2>&1
