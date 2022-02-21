

For ($i=0.7; $i -le 0.7; $i+=0.2) {
    For ($j=4; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 12 T     # Cars do their own routing
        Start-Sleep -Seconds 10
    }
}



For ($i=0.9; $i -le 1; $i+=0.2) {
    For ($j=1; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 12 T     # Cars do their own routing
        Start-Sleep -Seconds 10
        python main.py $i $j 12 10 1 4 2 10 4 F     # Without load balancing
        Start-Sleep -Seconds 10
    }
}

For ($i=0.3; $i -le 1; $i+=0.2) {
    For ($j=0; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 4 T     # Normal
        Start-Sleep -Seconds 10
    }
}

For ($i=0.3; $i -le 1; $i+=0.2) {
    For ($j=1; $j -le 4; $j++) {
        python main.py $i $j 12 10 1 4 2 10 1 T     # Centralized routing
        Start-Sleep -Seconds 60
    }
}
