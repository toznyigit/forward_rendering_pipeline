CC = g++
CFLAGS = -O3

rasterizer: *.o
	$(CC) $(CFLAGS) -o rasterizer *.o
	rm *.o

*.o: 
	$(CC) $(CFLAGS) -c *.cpp

clean:
	rm rasterizer

deleteppm:

	rm *.ppm
deletepng:
	rm *.png

delall:	deleteppm deletepng

all: $(file)*.xml
	for file in $^; do \
		./rasterizer $$file; \
	done
	
clearall: delall clean
