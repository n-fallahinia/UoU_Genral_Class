CXX = g++

# Include location for our header files
IDIR = include

# Define different directories
SDIR = src
ODIR = obj
LDIR = lib
BDIR = bin

# Include location for the Maglev driver
# This is stored in a separate variable because the -isystem flag is used, to
#   prevent warnings about the typedef declarations in the MLHD API files
MAGLEV_INC = include/maglev
OPENCV_INC = include/opencv

# Define flags for C++
#   We choose to have all warnings on to encourage us to use proper style
CXXFLAGS = $(patsubst %,-I%,$(IDIR)) $(patsubst %,-isystem%,$(MAGLEV_INC)) $(patsubst %,-isystem%,$(OPENCV_INC)) -Wall -ansi

# Include libraries for the Maglev driver
LIBS = -L$(LDIR) -lmlhi_api_linux
# Include libraries for FLTK
LIBS += `fltk-config --ldflags --cxxflags --use-images --use-gl`
# Include libraries for the Sensoray driver
LIBS += -s -lcomedi -lm -lc -lrt
# Include libraries for OpenGL
LIBS += -lGLU -lGL

# Set up dependencies
_DEPS = Constants
DEPS = $(patsubst %,$(IDIR)/%.h,$(_DEPS))

# Define object files
_OBJ = ForceSensor TimeHandler TimePlot BarPlot UserInterface WorkspacePlot MaglevControl functions main
OBJ = $(patsubst %,$(ODIR)/%.o,$(_OBJ))

# Define target output file
TARGET := $(BDIR)/main

# Rule to build all
all: $(TARGET)

# Rule to build regular .o files 
$(ODIR)/%.o: $(SDIR)/%.cpp $(patsubst %,$(IDIR)/%,%.h) $(DEPS)
	@echo Building $@
	$(CXX) -c -o $@ $< $(CXXFLAGS)

# Rules for individual .o files
$(ODIR)/functions.o: $(SDIR)/functions.cpp $(IDIR)/MaglevControl.h $(IDIR)/maglev_parameters.h $(DEPS)
	@echo Building $@
	$(CXX) -c -o $@ $< $(CXXFLAGS)

# Rules for individual .o files
$(ODIR)/MaglevControl.o: $(SDIR)/MaglevControl.cpp $(IDIR)/MaglevControl.h $(IDIR)/maglev_parameters.h $(DEPS)
	@echo Building $@
	$(CXX) -c -o $@ $< $(CXXFLAGS)

# Rule to build the target output file
$(TARGET): $(OBJ)
	@echo Building target output file
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

.PHONY: clean
clean:
	@echo cleaning objs
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
	rm -r $(TARGET)
