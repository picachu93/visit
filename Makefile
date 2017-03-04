CC = gcc
CFLAGS = -g -Wall -std=c99
all: visit 
visit: visit.o
	$(CC) $(CFLAGS) -o visit visit.o
visit.o: visit.c
	$(CC) $(CFLAGS) -c visit.c
clean: 
$(RM) $(all)
