This software use Ino tool to convert Arduino sketch into classic .hex object, ready to be used for a specifique board.


Requierements
==================
You need Ino Tool. Get and install with Python tool :

pip install ino

Link to InoTool: http://inotool.org/

Setup
==================

You first need to create an empty folder where init ino files. 
mkdir build
cd build
Then, create ino files and delete example sketch :
ino init && rm src/*.ino

Finally, copy your src file to ino source folder
cp ../src/* ./src/

Warning, your git repo is kinda fucked-up.


Compilation :
make BOARD_ID=<your_board_id>

Where <your_board_id> is the ID of the target board. Target specified in Makefile.

Upload:
make serial

Upload HEX file to the target board. Device specified in Makefile

Console:
make serial

Open a terminal link (picocom) between your target board and the desktop.

ctrl+a ctrl+q to quit picocom
