
AM_LDFLAGS=
AM_CFLAGS=

DEFS+= \
    -Icsrc -Icppsrc -Idogmsrc -Inonarduino \
    `getconf LFS_CFLAGS` \
    `getconf LFS64_CFLAGS` \
    -D_GNU_SOURCE \
    -D_FILE_OFFSET_BITS=64 \
    $(NULL)

AM_CFLAGS+= \
    -I$(top_builddir)/include/ \
    $(NULL)

AM_LDFLAGS += \
    -L$(top_builddir)/ \
    `getconf LFS_LDFLAGS` \
    `getconf LFS64_LDFLAGS` \
    $(NULL)

if DEBUG
# use "valgrind --tool=memcheck --leak-check=yes" to check memory leak, MemWatch will drag the program.
#DEFS+=-DMEMWATCH
DEFS+= -DDEBUG=1
AM_CFLAGS+=-g -Wall

else
AM_CFLAGS+=-O3 -Wall
endif

EXT_FLAGS=
@MK@GITNUMTMP=$(shell cd "$(top_srcdir)"; A=$$(git show | head -n 1 | awk '{print $$2}'); echo $${A:0:7}; cd - > /dev/null )
#@MK@SVNNUMTMP=$(shell cd "$(top_srcdir)"; LC_ALL=C svn info | grep -i Revision | awk '{print $$2}'; cd - > /dev/null )
#@MK@ifeq ($(SVNNUMTMP),)
#EXT_FLAGS+= -DSVN_VERSION='"${GITNUMTMP}"'
#@MK@else
#EXT_FLAGS+= -DSVN_VERSION='"${SVNNUMTMP}"'
#@MK@endif
@MK@ifeq ($(GITNUMTMP),)
@MK@else
EXT_FLAGS+= -DSVN_VERSION='"${GITNUMTMP}"'
@MK@endif
DEFS+=$(EXT_FLAGS)

SRC_BASE= \
    csrc/u8g_page.c \
    csrc/u8g_u16toa.c \
    csrc/u8g_u8toa.c \
    sfntsrc/u8g_font_data.c \
    sys/pbm/dev/u8g_dev_pbm.c \
    sys/pbm/dev/u8g_dev_pbm_gr_h2.c \
    sys/pbm/dev/u8g_dev_pbm_h.c \
    $(NULL)

SRC_COMMON_ALG= \
    csrc/u8g_bitmap.c \
    csrc/u8g_circle.c \
    csrc/u8g_clip.c \
    csrc/u8g_cursor.c \
    csrc/u8g_ellipse.c \
    csrc/u8g_font.c \
    csrc/u8g_line.c \
    csrc/u8g_pb14v1.c \
    csrc/u8g_pb16h1.c \
    csrc/u8g_pb16h2.c \
    csrc/u8g_pb16v1.c \
    csrc/u8g_pb16v2.c \
    csrc/u8g_pb32h1.c \
    csrc/u8g_pb8h1.c \
    csrc/u8g_pb8h1f.c \
    csrc/u8g_pb8h2.c \
    csrc/u8g_pb8h8.c \
    csrc/u8g_pb8v1.c \
    csrc/u8g_pb8v2.c \
    csrc/u8g_pb.c \
    csrc/u8g_pbxh16.c \
    csrc/u8g_pbxh24.c \
    csrc/u8g_polygon.c \
    csrc/u8g_rect.c \
    csrc/u8g_rot.c \
    csrc/u8g_scale.c \
    csrc/u8g_virtual_screen.c \
    $(NULL)

SRC_COMMON_DRV= \
    csrc/chessengine.c \
    csrc/u8g_com_api_16gr.c \
    csrc/u8g_com_api.c \
    csrc/u8g_com_arduino_attiny85_hw_spi.c \
    csrc/u8g_com_arduino_common.c \
    csrc/u8g_com_arduino_fast_parallel.c \
    csrc/u8g_com_arduino_hw_spi.c \
    csrc/u8g_com_arduino_hw_usart_spi.c \
    csrc/u8g_com_arduino_no_en_parallel.c \
    csrc/u8g_com_arduino_parallel.c \
    csrc/u8g_com_arduino_port_d_wr.c \
    csrc/u8g_com_arduino_ssd_i2c.c \
    csrc/u8g_com_arduino_st7920_custom.c \
    csrc/u8g_com_arduino_st7920_hw_spi.c \
    csrc/u8g_com_arduino_st7920_spi.c \
    csrc/u8g_com_arduino_std_sw_spi.c \
    csrc/u8g_com_arduino_sw_spi.c \
    csrc/u8g_com_arduino_t6963.c \
    csrc/u8g_com_arduino_uc_i2c.c \
    csrc/u8g_com_atmega_hw_spi.c \
    csrc/u8g_com_atmega_parallel.c \
    csrc/u8g_com_atmega_st7920_hw_spi.c \
    csrc/u8g_com_atmega_st7920_spi.c \
    csrc/u8g_com_atmega_sw_spi.c \
    csrc/u8g_com_i2c.c \
    csrc/u8g_com_io.c \
    csrc/u8g_com_msp430_hw_spi.c \
    csrc/u8g_com_null.c \
    csrc/u8g_com_std_sw_spi.c \
    csrc/u8g_delay.c \
    csrc/u8g_dev_a2_micro_printer.c \
    csrc/u8g_dev_flipdisc_2x7.c \
    csrc/u8g_dev_gprof.c \
    csrc/u8g_dev_ht1632.c \
    csrc/u8g_dev_ili9325d_320x240.c \
    csrc/u8g_dev_ks0108_128x64.c \
    csrc/u8g_dev_lc7981_160x80.c \
    csrc/u8g_dev_lc7981_240x128.c \
    csrc/u8g_dev_lc7981_240x64.c \
    csrc/u8g_dev_lc7981_320x64.c \
    csrc/u8g_dev_ld7032_60x32.c \
    csrc/u8g_dev_null.c \
    csrc/u8g_dev_pcd8544_84x48.c \
    csrc/u8g_dev_pcf8812_96x65.c \
    csrc/u8g_dev_sbn1661_122x32.c \
    csrc/u8g_dev_ssd1306_128x32.c \
    csrc/u8g_dev_ssd1306_128x64.c \
    csrc/u8g_dev_ssd1309_128x64.c \
    csrc/u8g_dev_ssd1322_nhd31oled_bw.c \
    csrc/u8g_dev_ssd1322_nhd31oled_gr.c \
    csrc/u8g_dev_ssd1325_nhd27oled_bw.c \
    csrc/u8g_dev_ssd1325_nhd27oled_bw_new.c \
    csrc/u8g_dev_ssd1325_nhd27oled_gr.c \
    csrc/u8g_dev_ssd1325_nhd27oled_gr_new.c \
    csrc/u8g_dev_ssd1327_96x96_gr.c \
    csrc/u8g_dev_ssd1351_128x128.c \
    csrc/u8g_dev_ssd1353_160x128.c \
    csrc/u8g_dev_st7565_64128n.c \
    csrc/u8g_dev_st7565_dogm128.c \
    csrc/u8g_dev_st7565_dogm132.c \
    csrc/u8g_dev_st7565_lm6059.c \
    csrc/u8g_dev_st7565_lm6063.c \
    csrc/u8g_dev_st7565_nhd_c12832.c \
    csrc/u8g_dev_st7565_nhd_c12864.c \
    csrc/u8g_dev_st7687_c144mvgd.c \
    csrc/u8g_dev_st7920_128x64.c \
    csrc/u8g_dev_st7920_192x32.c \
    csrc/u8g_dev_st7920_202x32.c \
    csrc/u8g_dev_t6963_128x128.c \
    csrc/u8g_dev_t6963_128x64.c \
    csrc/u8g_dev_t6963_240x128.c \
    csrc/u8g_dev_t6963_240x64.c \
    csrc/u8g_dev_tls8204_84x48.c \
    csrc/u8g_dev_uc1601_c128032.c \
    csrc/u8g_dev_uc1608_240x128.c \
    csrc/u8g_dev_uc1608_240x64.c \
    csrc/u8g_dev_uc1610_dogxl160.c \
    csrc/u8g_dev_uc1611_dogm240.c \
    csrc/u8g_dev_uc1611_dogxl240.c \
    csrc/u8g_dev_uc1701_dogs102.c \
    csrc/u8g_dev_uc1701_mini12864.c \
    csrc/u8g_ll_api.c \
    csrc/u8g_state.c \
    dogmsrc/u8g_dogm128_api.c \
    dogmsrc/spacetrash.c \
    cppsrc/U8glib.cpp \
    $(NULL)

SRC_WPI= \
    $(SRC_COMMON_ALG) \
    $(SRC_COMMON_DRV) \
    csrc/u8g_com_raspberrypi_hw_spi.c \
    csrc/u8g_com_raspberrypi_ssd_i2c.c \
    $(NULL)

SRC_SDL= \
    $(SRC_COMMON_ALG) \
    $(SRC_COMMON_DRV) \
    sys/sdl/dev/u8g_dev_sdl.c \
    $(NULL)

include_HEADERS = \
    $(top_srcdir)/csrc/u8g.h \
    $(top_srcdir)/cppsrc/U8glib.h \
    $(top_srcdir)/dogmsrc/u8g_dogm128_api.h \
    $(top_srcdir)/nonarduino/Printable.h \
    $(top_srcdir)/nonarduino/Print.h \
    $(top_srcdir)/nonarduino/WString.h \
    $(NULL)

noinst_HEADERS=

lib_LTLIBRARIES=libu8gbase.la libu8gsdl.la libu8gwpi.la

libu8gbase_la_SOURCES=$(SRC_BASE)

libu8gsdl_la_SOURCES=$(SRC_SDL)

libu8gwpi_la_SOURCES=$(SRC_WPI)

libu8gbase_la_CFLAGS=-Icsrc -Icppsrc -Inonarduino -DU8G_16BIT `sdl-config --cflags`
libu8gbase_la_CPPFLAGS=$(libu8gbase_la_CFLAGS)
libu8gbase_la_LDFLAGS=

libu8gsdl_la_CFLAGS=-Icsrc -Icppsrc -Inonarduino -DU8G_16BIT `sdl-config --cflags`
libu8gsdl_la_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
libu8gsdl_la_LDFLAGS=`sdl-config --libs`
libu8gsdl_la_LIBADD=$(top_builddir)/libu8gbase.la

libu8gwpi_la_CFLAGS=-Icsrc -Icppsrc -Inonarduino -DU8G_16BIT -DU8G_RASPBERRY_PI -DU8G_WITH_PINLIST
libu8gwpi_la_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
libu8gwpi_la_LDFLAGS=-lwiringPi
libu8gwpi_la_LIBADD=$(top_builddir)/libu8gbase.la

BIN_SDL= \
    u8gsdl_2bit u8gsdl_8bit u8gsdl_circle u8gsdl_helloworldpp \
    u8gsdl_cursor u8gsdl_fonttop u8gsdl_fullcolor u8gsdl_gabc u8gsdl_gah u8gsdl_greek \
    u8gsdl_helloworld u8gsdl_hicolor u8gsdl_logo u8gsdl_menu u8gsdl_polygon u8gsdl_xbm \
    u8gsdl_chess u8gsdl_spacetrash \
    u8gsdl_clip \
    $(NULL)

BIN_WPI=u8gwpi_chess u8gwpi_chessdogm u8gwpi_gtest u8gwpi_logo u8gwpi_spacetrash

BIN_TOOLS=bdf2u8g
bdf2u8g_SOURCES= \
    tools/font/bdf2u8g/bdf2u8g.c \
    $(NULL)
test:
	./bdf2u8g -f 2 tools/font/bdf/9x18.bdf u8g_aafont_9x18 test_u8g_aafont_9x18.c

bin_PROGRAMS=$(BIN_TOOLS) $(BIN_SDL)
noinst_PROGRAMS=$(BIN_WPI)

u8gsdl_clip_SOURCES=sys/pbm/cliptest/main.c
u8gsdl_clip_CFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_clip_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_2bit_SOURCES=sys/sdl/2bit/main.c
u8gsdl_2bit_CFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_2bit_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_8bit_SOURCES=sys/sdl/8bit/main.c
u8gsdl_8bit_CFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_8bit_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_chess_SOURCES=sys/sdl/chess/main.c
u8gsdl_chess_CFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_chess_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_circle_SOURCES=sys/sdl/circle/main.c
u8gsdl_circle_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_circle_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_helloworldpp_SOURCES=sys/sdl/cpp_helloworld/main.cpp
u8gsdl_helloworldpp_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_helloworldpp_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_cursor_SOURCES=sys/sdl/cursor/main.c
u8gsdl_cursor_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_cursor_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_fonttop_SOURCES=sys/sdl/FontPosTop/main.c
u8gsdl_fonttop_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_fonttop_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_fullcolor_SOURCES=sys/sdl/fullcolor/main.c
u8gsdl_fullcolor_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_fullcolor_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_gabc_SOURCES=sys/sdl/gabc/main.c
u8gsdl_gabc_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_gabc_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_gah_SOURCES=sys/sdl/gA_h/main.c
u8gsdl_gah_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_gah_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_greek_SOURCES=sys/sdl/greek/main.c
u8gsdl_greek_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_greek_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_helloworld_SOURCES=sys/sdl/helloworld/main.c
u8gsdl_helloworld_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_helloworld_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_hicolor_SOURCES=sys/sdl/hicolor/main.c
u8gsdl_hicolor_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_hicolor_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_logo_SOURCES=sys/sdl/cpp_logo/main.cpp
u8gsdl_logo_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_logo_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_menu_SOURCES=sys/sdl/menu/main.c
u8gsdl_menu_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_menu_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_polygon_SOURCES=sys/sdl/polygon/main.c
u8gsdl_polygon_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_polygon_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_spacetrash_SOURCES=sys/sdl/spacetrash/spacemain.c
u8gsdl_spacetrash_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_spacetrash_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

u8gsdl_xbm_SOURCES=sys/sdl/xbm/main.c
u8gsdl_xbm_CPPFLAGS=$(libu8gsdl_la_CFLAGS)
u8gsdl_xbm_LDFLAGS=-lu8gbase -lu8gsdl $(libu8gsdl_la_LDFLAGS)

.pde.cpp:
	cp $< $@
.ino.o:
	cp $< $@

#u8gwpi_logo_SOURCES=sys/arm/examples/u8g_logo/u8g_logo.c
#u8gwpi_logo_SOURCES=sys/atmega/u8g_logo/u8g_logo.c
u8gwpi_logo_SOURCES=nonarduino/wpimain.cpp sys/arduino/U8gLogo/U8gLogo.pde
u8gwpi_logo_CFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_logo_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_logo_LDFLAGS=-lu8gbase -lu8gwpi $(libu8gwpi_la_LDFLAGS)

u8gwpi_gtest_SOURCES=nonarduino/wpimain.cpp sys/arduino/GraphicsTest/GraphicsTest.pde
u8gwpi_gtest_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_gtest_LDFLAGS=-lu8gbase -lu8gwpi $(libu8gwpi_la_LDFLAGS)

u8gwpi_spacetrash_SOURCES=nonarduino/wpimain.cpp sys/arduino/SpaceTrashDogm/SpaceTrash.pde #sys/sdl/spacetrash/spacemain.c
u8gwpi_spacetrash_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_spacetrash_LDFLAGS=-lu8gbase -lu8gwpi $(libu8gwpi_la_LDFLAGS)

u8gwpi_chess_SOURCES=nonarduino/wpimain.cpp sys/arduino/Chess/Chess.pde
u8gwpi_chess_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_chess_LDFLAGS=-lu8gbase -lu8gwpi $(libu8gwpi_la_LDFLAGS)

u8gwpi_chessdogm_SOURCES=nonarduino/wpimain.cpp sys/arduino/ChessDogm/Chess.pde
u8gwpi_chessdogm_CPPFLAGS=$(libu8gwpi_la_CFLAGS)
u8gwpi_chessdogm_LDFLAGS=-lu8gwpi $(libu8gwpi_la_LDFLAGS)
