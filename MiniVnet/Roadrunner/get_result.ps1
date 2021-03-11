
rm result/*

For ($j=1; $j -le 10; $j+=1) {
	For ($i=0.7; $i -le 0.9; $i+=0.1) {
		python main.py $i $j 0 2
	}
}

For ($j=1; $j -le 10; $j+=1) {
	For ($i=0.7; $i -le 0.9; $i+=0.1) {
		python main.py $i $j 0 3
	}
}
