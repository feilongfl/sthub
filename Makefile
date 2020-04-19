
NPROC=$(shell nproc)

stm32g431:
	cd prj/stm32g431rbtx && make -j $(NPROC)
	mkdir -p build/stm32g431rbtx
	cp prj/stm32g431rbtx/build/sthub.elf build/stm32g431rbtx/
	cp prj/stm32g431rbtx/build/sthub.bin build/stm32g431rbtx/
	cp prj/stm32g431rbtx/build/sthub.map build/stm32g431rbtx/

stm32g431_clean:
	cd prj/stm32g431rbtx && make clean

all: stm32g431

clean: stm32g431_clean
	rm -rv build/*
