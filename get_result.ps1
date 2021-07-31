rm MiniVnet_cpp/result/*
rm SUMO_env_cpp/result/*
rm -rf result

For ($rSeed=0; $rSeed -le 4; $rSeed+=1) {
  For ($aRate=0.1; $aRate -le 0.7; $aRate+=0.2) {
    For ($gridSize=7; $gridSize -le 7; $gridSize+=1) {
      For ($iter=1; $iter -le 4; $iter+=1) {
        For ($var=0; $var -le 30; $var+=10) {
          cd 'D:\Google Drive\NYU\V2X\UDTA\paper_v2\code\MiniVnet_cpp'
          START .\x64\Release\MiniVnet_cpp.exe
          cd 'D:\Google Drive\NYU\V2X\UDTA\paper_v2\code\SUMO_env_cpp'
          python main.py $aRate $rSeed $gridSize 10 1 6 $iter $var
        }
      }
    }
  }
}


mkdir result
mkdir result/MiniVnet
mkdir result/SUMO
mv MiniVnet_cpp/result/* result/MiniVnet
mv SUMO_env_cpp/result/* result/SUMO
