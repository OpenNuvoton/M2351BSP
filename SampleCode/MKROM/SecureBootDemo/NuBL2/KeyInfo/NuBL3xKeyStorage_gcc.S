

.section _NuBL32KeyStorage,"a",%progbits

.global g_NuBL32KeyStart
.global g_NuBL32KeyEnd
.global g_NuBL32KeyHashStart
.global g_NuBL32KeyHashEnd

.global g_NuBL33KeyStart
.global g_NuBL33KeyEnd
.global g_NuBL33KeyHashStart
.global g_NuBL33KeyHashEnd

.align 4

g_NuBL32KeyStart:
    .incbin BIN_NUBL32
g_NuBL32KeyEnd:

g_NuBL32KeyHashStart:
    .incbin BIN_NUBL32_HASH
g_NuBL32KeyHashEnd:

g_NuBL33KeyStart:
    .incbin BIN_NUBL33
g_NuBL33KeyEnd:

g_NuBL33KeyHashStart:
    .incbin BIN_NUBL33_HASH
g_NuBL33KeyHashEnd: