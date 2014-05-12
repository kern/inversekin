CFLAGS = -g -DGL_GLEXT_PROTOTYPES -I./include -I/usr/X11/include -DOSX -Wno-deprecated -Wno-extra-tokens
LDFLAGS = -framework GLUT -framework OpenGL \
    -L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
    -lGL -lGLU -lm -lstdc++

.PHONY: run clean

all: inversekin

inversekin: main.o arm.o segment.o
	g++ $(CFLAGS) -o inversekin main.o arm.o segment.o $(LDFLAGS)

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp -o main.o
    
arm.o: arm.cpp
	g++ $(CFLAGS) -c arm.cpp -o arm.o
    
segment.o: segment.cpp
	g++ $(CFLAGS) -c segment.cpp -o segment.o

run:
	./inversekin

clean:
	/bin/rm -f *.o inversekin