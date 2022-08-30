CC := g++
CFLAGS := -std=c++11 -fpermissive -I"/usr/include/python3.8"

LDFLAGS := -L"/usr/lib/python3.8/" -L"/usr/lib/python3.8/config-3.8m-x86_64-linux-gnu/" -lpython3.8 -lm
TARGET := STE_PC_server

# $(wildcard *.cpp /xxx/xxx/*.cpp): get all .cpp files from the current directory and dir "/xxx/xxx/"
SRCS := $(wildcard *.cpp)
# $(patsubst %.cpp,%.o$(SRCS)): substitute all ".cpp" file name strings to ".o" file name strings
OBJS := $(patsubst %.cpp,%.o,$(SRCS))

all: $(TARGET)
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
%.o: %.cpp
	$(CC) $(CFLAGS) -c $<
clean:
	rm -rf $(TARGET) *.o
	
.PHONY: all clean

#// g++ -o embedded -std=c++11 -fpermissive -I/usr/include/python3.6 python_embedded_c.cpp -L/usr/lib/python3.6/ -L/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/ -lpython3.6; ./embedded