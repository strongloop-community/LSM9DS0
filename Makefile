
CURSES_CFLAGS=-I/usr/include/ncursesw

LIBS=-lm -lmongoc-1.0 -lbson-1.0 -lpaho-mqtt3c -lpaho-mqtt3a -lpaho-mqtt3as -lpaho-mqtt3cs  -L/usr/local/lib/
CURSES_LIBS=-lncurses
EX_LIBS=/usr/local/lib/
BSON_DIR=/usr/local/include/libbson-1.0    
MONGO_DIR=/usr/local/include/libmongoc-1.0
HEADERS = edison-9dof-i2c.h
OBJ = edison-9dof-i2c.o
BINS = sensors calibrate-mag calibrate-acc-gyro
CFLAGS=-I. -I$(BSON_DIR) -I$(MONGO_DIR) -Wall -L$(EX_LIBS)

all: $(BINS)

sensors: sensors.o $(OBJ)
	$(CC) -g -o $@ $^ $(CFLAGS) $(LIBS) -L$(EX_LIBS)

calibrate-acc-gyro: calibrate-acc-gyro.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

calibrate-mag: calibrate-mag.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(CURSES_CFLAGS) $(LIBS) $(CURSES_LIBS)


%.o: %.c $(HEADERS) 
	$(CC) -g -c -o $@ $< $(CFLAGS) -L$(EX_LIBS)

calibrate-mag.o: calibrate-mag.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CURSES_CFLAGS)


.PHONY: clean

clean:
	rm -f *.o $(BINS)
