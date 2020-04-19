
NPROC=16

stm32g431:
	cd prj/stm32g431rbtx && make -j $(NPROC)

stm32g431_clean:
	cd prj/stm32g431rbtx && make clean

all: stm32g431

clean: stm32g431_clean

