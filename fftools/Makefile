AVPROGS-$(CONFIG_FFMPEG)   += ffmpeg
AVPROGS-$(CONFIG_FFPLAY)   += ffplay
AVPROGS-$(CONFIG_FFPROBE)  += ffprobe
# -- vgtmpeg
AVPROGS-yes	+= vgtmpeg
# -- vgtmpeg



AVPROGS     := $(AVPROGS-yes:%=%$(PROGSSUF)$(EXESUF))
PROGS       += $(AVPROGS)

# --vgtmpeg
AVBASENAMES  = ffmpeg ffplay ffprobe vgtmpeg
FF_EXTRALIBS += -ldvdread -lbluray
# --vgtmpeg

ALLAVPROGS   = $(AVBASENAMES:%=%$(PROGSSUF)$(EXESUF))
ALLAVPROGS_G = $(AVBASENAMES:%=%$(PROGSSUF)_g$(EXESUF))

# --vgtmpeg
OBJS-ffmpeg                        += fftools/ffmpeg_opt.o fftools/ffmpeg_filter.o fftools/ffmpeg_hw.o fftools/vgtmpeg_support.o
OBJS-ffmpeg-$(CONFIG_CUVID)        += fftools/ffmpeg_cuvid.o
OBJS-ffmpeg-$(CONFIG_LIBMFX)       += fftools/ffmpeg_qsv.o
ifndef CONFIG_VIDEOTOOLBOX
OBJS-ffmpeg-$(CONFIG_VDA)          += fftools/ffmpeg_videotoolbox.o
endif
OBJS-ffmpeg-$(CONFIG_VIDEOTOOLBOX) += fftools/ffmpeg_videotoolbox.o

OBJS-vgtmpeg                        += fftools/ffmpeg_opt.o fftools/ffmpeg_filter.o fftools/ffmpeg_hw.o fftools/vgtmpeg_support.o
OBJS-vgtmpeg-$(CONFIG_CUVID)        += fftools/ffmpeg_cuvid.o
OBJS-vgtmpeg-$(CONFIG_LIBMFX)       += fftools/ffmpeg_qsv.o
ifndef CONFIG_VIDEOTOOLBOX
OBJS-vgtmpeg-$(CONFIG_VDA)          += fftools/ffmpeg_videotoolbox.o
endif
OBJS-vgtmpeg-$(CONFIG_VIDEOTOOLBOX) += fftools/ffmpeg_videotoolbox.o
# --vgtmpeg

define DOFFTOOL
OBJS-$(1) += fftools/cmdutils.o fftools/$(1).o $(OBJS-$(1)-yes)
$(1)$(PROGSSUF)_g$(EXESUF): $$(OBJS-$(1))
$$(OBJS-$(1)): | fftools
$$(OBJS-$(1)): CFLAGS  += $(CFLAGS-$(1)) 
$(1)$(PROGSSUF)_g$(EXESUF): LDFLAGS += $(LDFLAGS-$(1))
$(1)$(PROGSSUF)_g$(EXESUF): FF_EXTRALIBS += $(EXTRALIBS-$(1)) -ldvdread
-include $$(OBJS-$(1):.o=.d)
endef

$(foreach P,$(AVPROGS-yes),$(eval $(call DOFFTOOL,$(P))))

all: $(AVPROGS)

fftools/ffprobe.o fftools/cmdutils.o: libavutil/ffversion.h | fftools
OBJDIRS += fftools

ifdef AVPROGS
install: install-progs install-data
endif

install-progs-yes:
install-progs-$(CONFIG_SHARED): install-libs

install-progs: install-progs-yes $(AVPROGS)
	$(Q)mkdir -p "$(BINDIR)"
	$(INSTALL) -c -m 755 $(AVPROGS) "$(BINDIR)"

uninstall: uninstall-progs

uninstall-progs:
	$(RM) $(addprefix "$(BINDIR)/", $(ALLAVPROGS))

clean::
	$(RM) $(ALLAVPROGS) $(ALLAVPROGS_G) $(CLEANSUFFIXES:%=fftools/%)
