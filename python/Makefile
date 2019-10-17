LCM = lcm-gen

LCMTYPES_NAMES := $(shell cd ../lcmtypes && ls *.lcm)

# Construct the pathnames to the python types
LCMTYPES_PYTHON := $(LCMTYPES_NAMES:%.lcm=%.py)
LCMTYPES_PYTHON := $(addprefix mobilesim/lcmtypes/,$(LCMTYPES_PYTHON))

JAVAC = javac
LCM_JAR = /usr/local/share/java/lcm.jar

all: $(LCMTYPES_PYTHON)

mobilesim/lcmtypes/%.py: ../lcmtypes/%.lcm
	@echo "\t$<"
	@$(LCM) -p --package-prefix=mobilesim.lcmtypes $<
#@$(LCM) -p --ppath=mobilesim --package-prefix=lcmtypes $<

clean:
	@rm -rf *~ mobilesim/lcmtypes/*
