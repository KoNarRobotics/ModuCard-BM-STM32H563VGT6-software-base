# ModuCard-BM-STM32H563VGT6-software-base
Minimum software config for ModuCard base module board with  STM32H563VGT6 mcu.

# How to create software project for new ModuCard module.
## Creating your own module software repo.
1. Make  a fork of this repo on KoNarRobotics.
2. Name the repo as **ModuCard-MODULE_NAME-software** where MODULE_NAME is the name of your module.
3. Clone your forked repo to your local machine:
  ```bash
  git clone <YOUR FORKED REPO URL>
  ```
4. Cd into the cloned repo:
  ```bash
  cd ModuCard-MODULE_NAME-software
  ```
5. Add remote for the base project to be able to pull future updates:
  ```bash
  git remote add upstream git@github.com:KoNarRobotics/ModuCard-BM-STM32H563VGT6-software-base.git
  ```
6. Now you are ready to setup the project. 

## Setting up the pc to build the project.
For more info about setting up the project see **[StmEpic getting started](https://stmepic.d3lab.dev/md_docs_pages_geting_started.html)**
Go through the **Requirements** section.

## After cloning the repo.
1. Open the project in your VSCode.
2.  In **CMakeLists.txt** change the project name.:
```cmake
set(CMAKE_PROJECT_NAME YOUR_PROJECT_NAME_HERE)
```
3. To init all the submodules
  ```bash
  git submodule update --init --recursive
  ``` 

4. Build the project to make sure everything is working.
```bash
./compile.sh
```


## Writing your code for the module.
In the **src/module.cpp** file you can add your custom module initialization code in the `init_board()` function.
You can also add your custom code in there as needed.
Declaration can be added to **src/module.hpp**.
Callbacks should be added in **src/module_callbacks.cpp**.

You can always create your own files and include them in CMakeLists.txt .
**Do not modify files** like **main_prog.cpp** or **main_prog.hpp** unless you really know what you are doing.


# Configuring project
### Change project name

### In files:
- **.vscode/launch.json**  (debug config for vscode)
 in **"inputs" -> "id":"stlink_id" -> "default"** set to your stlink serial number of the board you want to program (only for local programing when you are physically connected to the board via usb adapter or if you are working over ssh wth the board from the OBC) 

-  **".vscode/tasks.json"** 
 in **"inputs" -> "stlinkID" -> "options"** you can add multiple stlink ids if you have multiple boards with the same software, You can add them some fancy names to recognize them easel.
