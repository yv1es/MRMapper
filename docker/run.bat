@echo off 
echo Running container
Rem start cmd /C python ../host/stream_camera.py
docker run -it -p 5000:5000 -p 5001:5001 -p 5002:5002 -p 5003:5003 -p 5004:5004 -e DISPLAY=host.docker.internal:0.0 --gpus all mrmapper-plane-seg bash
