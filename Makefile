CC=gcc

all: controller.out plant.out attacker.out


plant.out: plant.o matrix.o log_manager.o udp_protocol.o
	gcc -o plant.out plant.o matrix.o udp_protocol.o log_manager.o -lrt

controller.out: controller.o matrix.o log_manager.o udp_protocol.o queue.o
	gcc -o controller.out controller.o matrix.o udp_protocol.o log_manager.o queue.o

attacker.out: attacker.o
	gcc -o attacker.out attacker.o -lrt

controller.o: matrix.h udp_protocol.h log_manager.h queue.h controller.c
	gcc -c -o controller.o controller.c

plant.o: matrix.h udp_protocol.h log_manager.h controller.c
	gcc -c -o plant.o plant.c -lrt

matrix.o: matrix.h matrix.c
	gcc -c -o matrix.o matrix.c

udp_protocol.o: udp_protocol.h udp_protocol.c
	gcc -c -o udp_protocol.o udp_protocol.c

log_manager.o: log_manager.h log_manager.c
	gcc -c -o log_manager.o log_manager.c

queue.o : queue.h queue.c
	gcc -c -o queue.o queue.c

attacker.o : attacker.c
	gcc -c -o attacker.o attacker.c -lrt

clean: 
	rm -f *.o
	rm -f controller.out plant.out
