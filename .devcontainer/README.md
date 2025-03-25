# Development Container Usage
## Requirements
- VSCode
- VSCode Remote Development Extension
- Docker CLI
- Docker Desktop (recommended)

## Install VSCode
Go to https://code.visualstudio.com/ and follow the instructions
## Install the Remote Development Extension
1. Open VSCode
2. Click on the extensions button on the left tool bar.
3. Search for Remote Development
4. Click the install button
## Install Docker CLI and Desktop
Go to https://docs.docker.com/desktop/ and follow the instructions

## Using the Container
1. Open the project folder in VSCode
2. Press Ctrl+Shift+P to open the command menu
3. Type ```Dev Containers: Reopen in Container```
4. Press enter, the Docker image will build and the container will open the project.

The entire project will be available in the ```doc``` folder. 
All source code should be placed in the ```src``` directory to meet catkin's expectations. Files placed in ```src``` are not tracked by git at this time.
