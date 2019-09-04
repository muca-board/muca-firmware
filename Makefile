# O (Output) is the build directory
ifeq '$O' ''
O = .
endif
# V (Verbosity) is 0 (quiet) or 1 (verbose)
ifeq '$V' ''
V = 0
endif
# D (Debug) is 0 (optimize) or 1 (build debug version)
ifeq '$D' ''
D = 0
endif

# files
SRC_BIN = $(wildcard *.c)
OBJS = $(OBJ_BIN)
OBJ_BIN = $(SRC_BIN:%.c=$O/%.o)
TARGET_BIN = $O/muca-flash
BINS = $(TARGET_BIN)

DESTDIR ?= /usr

# options
override CFLAGS += -fPIC -Wall
override LDFLAGS +=

# first rule (default)
all:

# rules verbosity
ifneq '$V' '0'
P = @ true
E =
else
P = @ echo
E = @
endif

# rules
directories :
	$P '  MKDIR'
	$E mkdir -p $O

$O/%.o : %.c directories
	$P '  CC      $(@F)'
	$E $(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(TARGET_BIN): $(OBJ_BIN)
	$P '  LD      $(@F)'
	$E $(CC) $(LDFLAGS) $^ -o $@

.PHONY : all
all : $(TARGET_BIN)

.PHONY: clean
clean:
	$P '  RM      objs bins'
	$E rm -f $(OBJS) $(BINS)

.PHONY: install
install: all
	$P '  INSTALL DIRS'
	$E mkdir -p $(DESTDIR)/bin
	$P '  INSTALL FILES'
	$E install $(TARGET_BIN) $(DESTDIR)/bin

.PHONY: uninstall
uninstall:
	$P '  UNINSTALL'
	$E rm -f $(DESTDIR)/bin/$(notdir $(TARGET_BIN))

.PHONY: help
help:
	@echo
	@echo make [D=1] [V=1] [O=path]
	@echo "   D=1: build debug version (default: D=0)"
	@echo "   V=1: verbose output (default: V=0)"
	@echo "   O=path: build binary in path (default: O=.)"
	@echo
