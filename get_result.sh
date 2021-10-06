rm MiniVnet_cpp/result/*
rm SUMO_env_cpp/result/*
rm -rf result

mkdir MiniVnet_cpp/result
mkdir SUMO_env_cpp/result

export LD_LIBRARY_PATH=/local/mwang/ortools/lib

for rSeed in 2
do
for aRate in $(seq 0.1 0.025 0.7)
do
for gridSize in 7
do

for iter in 1
do
for var in 10
do
echo ....
echo "Time ERR" $var "| Iteration" $iter $aRate $rSeed $gridSize
echo ....
( cd SUMO_env_cpp ; python3 main.py $aRate $rSeed $gridSize 10 1 6 $iter $var)
sleep 5
pkill main
pkill python3
done
done

done
done
done


mkdir result
mkdir result/MiniVnet
mkdir result/SUMO
mv MiniVnet_cpp/result/* result/MiniVnet
mv SUMO_env_cpp/result/* result/SUMO
