# Nao SLAM Monitor
This tool is developed for controlling the Nao Robot and performing the landmark detection at the same.

![](https://drive.google.com/uc?authuser=0&id=1G4_BtnTECsw8c7uf9GFN67E5tJ7LHmOb&export=download)

## Environment requirement
- Python 2.7
- Python Naoqi SDK
- OpenCV

For a detailed list for all the package versions used, please check **requirements.txt**.

## Run
1. set `NAO_IP`, `NAO_PORT` and `LOG_FILE` in **main.py**

2. `python main.py`

## Keyboard control
    w: move forward
    s: move backward
    a: turn left
    d: turn right
    q: rest
    e: standup
    x: stop
    m: mark
    Control-q: quit

## Log format
At most one landmark can be detected.

### No landmark detected
    [time] x, y, θ, 0

### One landmark detected
    [time] x, y, θ, 1, landmark_number, range, bearing
