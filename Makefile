
NPROC=16

stm32g431:
	cd prj/stm32g431rbtx && make -j $(NPROC)

all: stm32g431

clean:

