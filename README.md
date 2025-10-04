# ModuCard-BM-STM32H563VGT6-software-base
Minimum software config for ModuCard base module board with  STM32H563VGT6 mcu.

## After cloning the repo
1. Run:
  ```bash
  git submodule update --init --recursive
  ```
  to clone all the submodules.
  **For more info about setting up the project see [StmEpic getting started](https://stmepic.d3lab.dev/md_docs_pages_geting_started.html)**

2. Change the git remote url to your own repo if you cloned the base project:
  ```bash
  git remote set-url origin <YOUR REPO URL>
  ``` 


# Configuring project
### Change project name
In **CMakeLists.txt** change the project name in the first line:
```cmake
set(CMAKE_PROJECT_NAME YOUR_PROJECT_NAME_HERE)
```

### In files:
- **.vscode/launch.json**  (debug config for vscode)
 in **"inputs" -> "id":"stlink_id" -> "default"** set to your stlink serial number of the board you want to program (only for local programing when you are physically connected to the board via usb adapter or if you are working over ssh wth the board from the OBC) 

-  **".vscode/tasks.json"** 
 in **"inputs" -> "stlinkID" -> "options"** you can add multiple stlink ids if you have multiple boards with the same software, You can add them some fancy names to recognize them easel.
