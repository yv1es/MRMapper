@echo off 
echo Running container
Rem start cmd /C python ../host/stream_camera.py
docker run -it -p 5000:5000 -p 5001:5001 -p 5002:5002 -e DISPLAY=host.docker.internal:0.0 --gpus all rtabmap-gpu-o3d bash
