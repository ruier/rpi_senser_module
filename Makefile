CC = gcc

default:
	$(CC) -c bcm2835.c
	$(CC) -c MAG3110/mag3110.c
	$(CC) -c ./MPL3115/mpl3115a2.c
	$(CC) -c ./MMA8491Q/mma8491q.c
clean:
	rm -rf *.o bcm ./MPL3115/mpl3115a2.o ./MMA8491Q/*.o

sensor:
	$(CC) -shared -o sensor.so bcm2835.o mpl3115a2.o mma8491q.o mag3110.o

