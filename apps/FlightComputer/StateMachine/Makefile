include $(APPDIR)/Make.defs

PROGNAME = $(CONFIG_FLIGHTCOMPUTER_STATEMACHINE_PROGNAME)
PRIORITY = $(CONFIG_FLIGHTCOMPUTER_STATEMACHINE_PRIORITY)
STACKSIZE = $(CONFIG_FLIGHTCOMPUTER_STATEMACHINE_STACKSIZE)
MODULE = $(CONFIG_FLIGHTCOMPUTER_STATEMACHINE)

# State Machine
MAINSRC = StateMachine.cxx
BASEDIR = $(APPDIR)/FlightComputer/StateMachine

# Subdirectories
SENSORDIR = $(BASEDIR)/sensors

# Shared Directories and Library Linking
SHAREDIR = $(APPDIR)/FlightComputer/FlightLib

# Include Sensors and Build
SENSORSRC = $(wildcard $(SENSORDIR)/**/*.cxx)

# Add Sensors to the CXX Compiler
CXXSRCS += $(SENSORSRC)

VPATH += $(SENSORDIR)
VPATH += $(SRCDIR)
VPATH += $(SHAREDIR)

INCLUDES += -I$(APPDIR)/FlightComputer/FlightLib

CXXFLAGS += $(INCLUDES)

# Link against FlightLib
LDLIBS += -L$(APPDIR)/FlightComputer/FlightLib -lflightlib

include $(APPDIR)/Application.mk