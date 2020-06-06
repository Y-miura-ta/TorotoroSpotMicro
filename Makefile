CC  = g++
CFLAGS    = -g -DPIC -shared -lboost_python3 -I /usr/include/python3.7m
OBJS      = Kalman.so
PROGRAMS   = Kalman.cpp Kalmanw.cpp

$(OBJS):
	$(CC) $(CFLAGS) -o $(OBJS) $(PROGRAMS)

all:clean $(OBJS)

clean:
	rm -f *.so
