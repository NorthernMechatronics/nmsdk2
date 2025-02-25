# This is an example Makefile to show how to build the library

COMPILER_ROOT = 
CMSIS_ROOT = ../../../../cmsis

# Compilation tools
CC := arm-none-eabi-gcc
ARMAR := arm-none-eabi-gcc-ar

# Compilation flags (here for Cortex-M4)
CFLAGS := -mcpu=cortex-m4 \
 -Wsign-compare \
 -Wdouble-promotion \
 -Ofast -ffast-math \
 -DNDEBUG \
 -Wall -Wextra  -Werror \
 -mfpu=fpv4-sp-d16 \
 -mfloat-abi=hard \
 -fdata-sections \
 -ffunction-sections


# Path to CMSIS_6
CMSIS_6 := $(CMSIS_ROOT)/CMSIS_6

# Path to CMSIS_DSP
CMSIS_DSP := $(CMSIS_ROOT)/CMSIS-DSP

# Path to CMSIS Core includes for Cortex-M
# For low end Cortex-A, use Core_A
# For high end Cortex-A (aarch64), don't use CMSIS 
# Core Includes (Refer to the CMSIS-DSP README to 
# know how to build in that case)
CMSIS_CORE_INCLUDES := $(CMSIS_6)/CMSIS/Core/Include 

# Sources
SRCS := \
$(CMSIS_DSP)/Source/BasicMathFunctions/BasicMathFunctions.c \
$(CMSIS_DSP)/Source/CommonTables/CommonTables.c \
$(CMSIS_DSP)/Source/InterpolationFunctions/InterpolationFunctions.c \
$(CMSIS_DSP)/Source/BayesFunctions/BayesFunctions.c \
$(CMSIS_DSP)/Source/MatrixFunctions/MatrixFunctions.c \
$(CMSIS_DSP)/Source/ComplexMathFunctions/ComplexMathFunctions.c \
$(CMSIS_DSP)/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
$(CMSIS_DSP)/Source/ControllerFunctions/ControllerFunctions.c \
$(CMSIS_DSP)/Source/SVMFunctions/SVMFunctions.c \
$(CMSIS_DSP)/Source/DistanceFunctions/DistanceFunctions.c \
$(CMSIS_DSP)/Source/StatisticsFunctions/StatisticsFunctions.c \
$(CMSIS_DSP)/Source/FastMathFunctions/FastMathFunctions.c \
$(CMSIS_DSP)/Source/SupportFunctions/SupportFunctions.c \
$(CMSIS_DSP)/Source/FilteringFunctions/FilteringFunctions.c \
$(CMSIS_DSP)/Source/TransformFunctions/TransformFunctions.c \
$(CMSIS_DSP)/Source/WindowFunctions/WindowFunctions.c

# Includes
DSP_INCLUDES := $(CMSIS_DSP)/Include \
  $(CMSIS_DSP)/PrivateInclude 

# Compilation flags for include folders
INC_FLAGS := $(addprefix -I,$(DSP_INCLUDES))
INC_FLAGS += $(addprefix -I,$(CMSIS_CORE_INCLUDES))
CFLAGS += $(INC_FLAGS) -DDISABLEFLOAT16 -DARM_MATH_LOOPUNROLL

# Output folder for build products
BUILDDIR := ./builddir

OBJ1 := $(SRCS:%.c=$(BUILDDIR)/%.o)
OBJ2 := $(notdir $(OBJ1))
OBJECTS := $(OBJ2:%=$(BUILDDIR)/%)

TARGET := $(BUILDDIR)/libCMSISDSP.a

# Build rules
all: $(TARGET)

install: $(TARGET)
	cp $< ./lib/libCMSISDSP.a

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(TARGET): $(BUILDDIR) $(OBJECTS)
	$(ARMAR) -rc $@ $(OBJECTS)
	
$(BUILDDIR)/BasicMathFunctions.o: $(CMSIS_DSP)/Source/BasicMathFunctions/BasicMathFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/CommonTables.o: $(CMSIS_DSP)/Source/CommonTables/CommonTables.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/InterpolationFunctions.o: $(CMSIS_DSP)/Source/InterpolationFunctions/InterpolationFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/BayesFunctions.o: $(CMSIS_DSP)/Source/BayesFunctions/BayesFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/MatrixFunctions.o: $(CMSIS_DSP)/Source/MatrixFunctions/MatrixFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/ComplexMathFunctions.o: $(CMSIS_DSP)/Source/ComplexMathFunctions/ComplexMathFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/QuaternionMathFunctions.o: $(CMSIS_DSP)/Source/QuaternionMathFunctions/QuaternionMathFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/ControllerFunctions.o: $(CMSIS_DSP)/Source/ControllerFunctions/ControllerFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/SVMFunctions.o: $(CMSIS_DSP)/Source/SVMFunctions/SVMFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/DistanceFunctions.o: $(CMSIS_DSP)/Source/DistanceFunctions/DistanceFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/StatisticsFunctions.o: $(CMSIS_DSP)/Source/StatisticsFunctions/StatisticsFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/FastMathFunctions.o: $(CMSIS_DSP)/Source/FastMathFunctions/FastMathFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/SupportFunctions.o: $(CMSIS_DSP)/Source/SupportFunctions/SupportFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/FilteringFunctions.o: $(CMSIS_DSP)/Source/FilteringFunctions/FilteringFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/TransformFunctions.o: $(CMSIS_DSP)/Source/TransformFunctions/TransformFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(BUILDDIR)/WindowFunctions.o: $(CMSIS_DSP)/Source/WindowFunctions/WindowFunctions.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

clean:
	rm -rf $(BUILDDIR)
