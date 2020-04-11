# calls:
CC         = g++
CFLAGS     = -c -Wall -O3
LDFLAGS    = 
EXECUTABLE = previz

SOURCES    = previz.cpp skeleton.cpp motion.cpp displaySkeleton.cpp
OBJECTS    = $(SOURCES:.cpp=.o)

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o previz
