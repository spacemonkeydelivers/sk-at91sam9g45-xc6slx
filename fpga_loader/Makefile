PROJECT = fpga_loader

#CC = arm-linux-gcc
#CP = arm-linux-g++

CC=arm-at91-linux-gnueabi-gcc
CP=arm-at91-linux-gnueabi-g++
LD=arm-at91-linux-gnueabi-ld

#LDFLAGS=-L/usr/local/cross-gcc/arm-2007q1/arm-linux/arm-linux/lib -static -s
#LDFLAGS=-L/usr/local/cross-gcc/arm-2007q1/arm-linux/arm-linux/lib

#export CC
#export CXX
#export LD
#export LDFLAGS



MKDIR = mkdir -p

#OBJDIR=.obj
OBJDIR=.

LDFLAGS += -lpthread -static -s
#CFLAGS +=`wx-config --cflags`
#LDFLAGS +=`wx-config --libs

SRCDIR = $(ROOTDIR)
SRC = main.c at91gpio.c
OBJ = $(patsubst %.c, $(OBJDIR)/%.o, $(SRC))
# executable
EXECUTABLE = $(OBJDIR)/$(PROJECT)

#copy: $(SRC) 
#	cp $(SRC) $(BUILDDIR)

all: $(OBJ)
	@echo building target file $(EXECUTABLE)
	@$(CC) -o $(EXECUTABLE) $(OBJ) ${CFLAGS} ${LDFLAGS}
	@echo target file $(EXECUTABLE) built successfully

$(OBJDIR):
	@-if test -d $(OBJDIR) mkdir $(OBJDIR)

$(OBJDIR)/%.o: %.c
	@echo compiling file $<
	@$(MKDIR) $(OBJDIR)
	@$(CP) -c ${CFLAGS} $< -o$@
#	@$(CC) -c ${CFLAGS} $< -o$@
	@echo file $< successfully compiled
clean:
	@rm -f $(OBJDIR)/*.o
	@rm -f $(EXECUTABLE)
#	@rmdir $(OBJDIR)

.PHONY: clean
