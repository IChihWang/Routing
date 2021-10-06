rm MiniVnet_cpp/result/*
rm SUMO_env_cpp/result/*
rm -rf result

cd 'D:\Google Drive\NYU\V2X\UDTA\paper_v2\code\MiniVnet_cpp'
START .\x64\Release\MiniVnet_cpp.exe
cd 'D:\Google Drive\NYU\V2X\UDTA\paper_v2\code\SUMO_env_cpp'
python main.py 0.8 1 4 10 1 6 1 10


mkdir result
mkdir result/MiniVnet
mkdir result/SUMO
mv MiniVnet_cpp/result/* result/MiniVnet
mv SUMO_env_cpp/result/* result/SUMO
