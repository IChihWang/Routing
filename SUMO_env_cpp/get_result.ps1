

For ($i=0.5; $i -le 0.5; $i+=0.2) {
    For ($j=4; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 1 T     # Centralized routing
    }
}


For ($i=0.7; $i -le 1; $i+=0.2) {
    For ($j=1; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 1 T     # Centralized routing
    }
}
