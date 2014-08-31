
TARGET = ${DEVICE}
BOARD_TYPE = mega2560
COMPILER = ino
SLAVE_ID=-1

build:
	${COMPILER} build -m ${BOARD_TYPE} -f=-DSLAVE_ID=${SLAVE_ID}

clean:
	${COMPILER} clean

up:
	${COMPILER} upload -m ${BOARD_TYPE} -p ${TARGET}

serial:
	${COMPILER} serial -b 115200 -p ${TARGET}
