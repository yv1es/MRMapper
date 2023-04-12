@echo off 
echo Running container
docker run -it -p 5000:5000 -e DISPLAY=host.docker.internal:0.0 --gpus all rtabmap bash