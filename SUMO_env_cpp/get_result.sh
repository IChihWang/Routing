rm result/*
#rm -rf result

export LD_LIBRARY_PATH=/localhome/michael_jr/ortools/lib


for i in $(seq 0.9 0.2 1.0)
do
for j in 0
do
#pkill sumo
#python3 main.py $i $j 12 10 1 4 2 10 4 T
#sleep 10
#pkill sumo
#python3 main.py $i $j 12 10 1 4 2 10 1 T
#sleep 60
#pkill sumo
python3 main.py $i $j 12 10 1 4 2 10 12 T
sleep 10
pkill sumo
pkill python3
pkill main
python3 main.py $i $j 12 10 1 4 2 10 4 F
sleep 10

done
done


