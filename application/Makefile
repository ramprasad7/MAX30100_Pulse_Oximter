all:pulse_application

pulse_application:filter.o main.o
	gcc -o pulse_application filter.o main.o -lm

filter.o:filter.c filter.h
	gcc -c filter.c -I. -lm 

main.o:main.c filter.h
	gcc  -c main.c -I. -lm 

clean:
	rm -f *.o pulse_application