# MIT License
#
# Copyright (c) 2026 Adrian Port
# See LICENSE for full license text.

USE_lgpio ?= 1
MO := mo

CC ?= gcc
AR ?= ar
SYS := $(shell $(CC) -dumpmachine)
PIMODEL := $(shell test -r /proc/cpuinfo && grep Model /proc/cpuinfo | sed -e 's/^.*Raspberry Pi //' -e 's/ .*//' || echo 0)

GPIOCHIP ?= 4
ifeq ($(PIMODEL),5)
  USE_HAT2 ?= 1
  USE_IIO ?= 1
else
  ifeq (, $(findstring redhat,$(SYS)))
    $(warning PI Models before 5 are not supported by this Makefile)
  endif
endif

BUILD := build
OBJDIR := $(BUILD)/obj
LIBDIR := $(BUILD)/lib
BINDIR := .
HDIR := $(shell pwd)
TDIR := ${HDIR}/tools
VENV := ${HOME}/.venv
PYTHON := ${VENV}/bin/python3
PIP := ${VENV}/bin/pip

INCLUDES := -Iinclude -Iinclude/eve -Iinclude/vibsense

COMMON_CFLAGS := -g -Wall -Wno-format-truncation -Wno-write-strings -fmessage-length=0 \
  -Wno-uninitialized -Werror=uninitialized -Wno-sign-compare -Werror=strict-aliasing \
  -fvisibility=hidden -Wno-maybe-uninitialized -Wno-strict-aliasing \
  -DDEBUG -Wformat -DLINUX -Wno-unknown-pragmas $(INCLUDES)

LIBS := -lm -ldl -lpthread -lconfig
FTLIBS :=

ifneq (, $(findstring redhat,$(SYS)))
  COMMON_CFLAGS += -DX86_64 -DPIMODEL=0
else ifneq (, $(findstring arm-linux-gnueabihf,$(SYS)))
  CC := /usr/bin/arm-linux-gnueabihf-gcc
  COMMON_CFLAGS += -DARM -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard -DPIMODEL=$(PIMODEL)
  LIBS += -liio
else ifneq (, $(findstring aarch64-linux-gnu,$(SYS)))
  COMMON_CFLAGS += -DARM -D__linux__ -DPIMODEL=$(PIMODEL)
  ifdef USE_lgpio
    LIBS += -llgpio
    COMMON_CFLAGS += -DUSE_LGPIO=1 -DGPIOCHIP=$(GPIOCHIP)
    ifdef USE_IIO
      LIBS += -liio
    endif
  else
    LIBS += -lwiringPi
  endif
endif

ifdef USE_HAT2
  COMMON_CFLAGS += -DUSE_HAT2=1
  HAT2_OBJECTS := $(OBJDIR)/mo/ad5592.o
endif

FTLIBS += -lftdi1 -Wl,-L/usr/local/lib
COMMON_CFLAGS += -DLIBFTDI1=1

CFLAGS ?= $(COMMON_CFLAGS)
LDFLAGS ?=

MO_SRCS := src/mo/mo.c src/mo/tts.c src/mo/lidar.c src/mo/vibsense_fake_window.c src/mo/vibsense_live.c
MO_OBJS := $(MO_SRCS:src/%.c=$(OBJDIR)/%.o) $(HAT2_OBJECTS)
EVE_SRCS := src/eve/eve_xvg.c src/eve/eve_lib.c src/eve/usb_bridge.c
EVE_OBJS := $(EVE_SRCS:src/%.c=$(OBJDIR)/%.o)
VIB_SRCS := src/vibsense/vibsense.c
VIB_OBJS := $(VIB_SRCS:src/%.c=$(OBJDIR)/%.o)

EVELIB := $(LIBDIR)/lib_eve.a
VIBLIB := $(LIBDIR)/libvibsense.a

.PHONY: all clean libs tools test dirs logs alllogs cleanlogs cleanrun
all: dirs $(MO) dl make_jpg
libs: $(EVELIB) $(VIBLIB)
tools: dl load make_jpg makeimage

dirs:
	@mkdir -p $(OBJDIR)/mo $(OBJDIR)/eve $(OBJDIR)/vibsense $(OBJDIR)/tools $(LIBDIR) runtime/logs/test runtime/screenshots config

$(OBJDIR)/%.o: src/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/tools/%.o: tools/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(EVELIB): $(EVE_OBJS) | dirs
	$(AR) rcs $@ $^

$(VIBLIB): $(VIB_OBJS) | dirs
	$(AR) rcs $@ $^

$(MO): $(MO_OBJS) $(EVELIB) $(VIBLIB) | dirs
	$(CC) $(MO_OBJS) $(EVELIB) $(VIBLIB) -Wall $(FTLIBS) $(LIBS) -o $@ $(LDFLAGS)

dl: $(OBJDIR)/tools/dl.o | dirs
	$(CC) $< -o $@ $(LIBS)

load: $(OBJDIR)/tools/load.o | dirs
	$(CC) $< -o $@ $(LIBS)

include/cinzel_bold_ttf.h: assets/fonts/cinzel/static/Cinzel-Bold.ttf
	xxd -i $< | sed '1s/.*/static const unsigned char cinzel_bold_ttf[] = {/' | sed 's/^unsigned int .*_len = /static const unsigned int cinzel_bold_ttf_len = /' > $@

make_jpg: tools/make_jpg.c include/cinzel_bold_ttf.h | dirs
	$(CC) $(CFLAGS) $< -o $@ -Wall -Wextra $$(pkg-config --cflags --libs freetype2) -ljpeg

makeimage: tools/makeimage.c | dirs
	$(CC) $(CFLAGS) $< -o $@ -Wall -Wextra $$(pkg-config --cflags --libs freetype2)

test: dirs $(VIBLIB)
	$(CC) $(CFLAGS) tests/vibsense/test_vibsense.c $(VIBLIB) -o $(BUILD)/test_vibsense -lm
	$(BUILD)/test_vibsense

LOG_HTML := efi_load_report.html
FORMAT := html
LOGDIR := runtime/logs
SSDIR := runtime/screenshots
INJSCALE ?=
OFFSET ?=
SCOPE ?=

ifneq ($(OFFSET),)
  OFFSET_ARG := --offset $(OFFSET)
else
  OFFSET_ARG :=
endif
ifneq ($(SCOPE),)
  SCOPE_ARG := --scope $(SCOPE)
else
  SCOPE_ARG :=
endif
ifneq ($(INJSCALE),)
  INJSCALE_ARG := --injscale $(INJSCALE)
else
  INJSCALE_ARG :=
endif

ELR_ARGS := ${INJSCALE_ARG} ${OFFSET_ARG} ${SCOPE_ARG}

# Look for any log directories that have not been processed, and make the html reports
logs: ${PYTHON}
	@for d in ${LOGDIR}/2*; do \
	  [ -d "$$d" ] || continue; \
	  if [ ! -f "$$d/${LOG_HTML}" ]; then \
	    echo "$$d:"; \
	    cd $$d && ${HDIR}/dl && \
		${PYTHON} ${TDIR}/efi_plot.py ${INJSCALE} && \
	        ${PYTHON} ${TDIR}/efi_load_report.py ${ELR_ARGS} --format ${FORMAT} && \
		cd ${HDIR}; \
	  fi; \
	done

# Look for any log directories that have not been processed, and make the reports
# Create html, pdf and docx output
alllogs: ${PYTHON}
	@for d in ${LOGDIR}/2*; do \
	  [ -d "$$d" ] || continue; \
	  if [ ! -f "$$d/${LOG_HTML}" ]; then \
	    echo "$$d:"; \
	    cd $$d && ${HDIR}/dl && \
		${PYTHON} ${TDIR}/efi_plot.py ${INJSCALE} && \
	        ${PYTHON} ${TDIR}/efi_load_report.py ${ELR_ARGS} --format html && \
	        ${PYTHON} ${TDIR}/efi_load_report.py ${ELR_ARGS} --format pdf && \
	        ${PYTHON} ${TDIR}/efi_load_report.py ${ELR_ARGS} --format docx && \
		cd ${HDIR}; \
	  fi; \
	done

# Clean log analysis results, leave the logs there
cleanlogs:
	@for d in ${LOGDIR}/2*; do \
	  [ -d "$$d" ] || continue; \
	  rm -f "$$d"/*.html "$$d"/*.pdf "$$d"/*.docx; \
	  rm -f "$$d"/*.dat; \
	done

# Clean all runtime things
cleanrun:
	@for d in ${LOGDIR}/2*; do \
	  [ -d "$$d" ] || continue; \
	  rm -rf "$$d"; \
	done
	@rm -f ${SSDIR}/*
	@rm -f runtime/*.log

${PYTHON}:
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install numpy plotly kaleido reportlab odfpy python-docx

clean:
	rm -rf $(BUILD)
	rm -f $(MO) dl load make_jpg makeimage
	rm -f logs/test/*
