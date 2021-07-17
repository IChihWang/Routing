rm MiniVnet_cpp/result/*
rm SUMO_env_cpp/result/*
rm -rf result

export LD_LIBRARY_PATH=/local/mwang/ortools/lib

for rSeed in $(seq 0 1 9)
do
for aRate in $(seq 0.1 0.1 0.8)
do
for gridSize in 2 5 7
do

for var in 0 1 2 3 4
do
echo ....
echo "Choose_car_option:" $var "|" $aRate $rSeed $gridSize
echo ....
( cd MiniVnet_cpp ; ./x64/Release/main) &
( cd SUMO_env_cpp ; python3 main.py $aRate $rSeed $gridSize 10 $var 4 1 10)
sleep 5
pkill main
pkill python3
done	

done	
done
done	


mkdir result
mkdir result/MiniVnet
mkdir result/SUMO
mv MiniVnet_cpp/result/* result/MiniVnet
mv SUMO_env_cpp/result/* result/SUMO
