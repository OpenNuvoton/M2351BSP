

    AREA _fwinfo, DATA, READONLY

    EXPORT  g_InfoDataBase
    EXPORT  g_InfoDataLimit

    ALIGN   4
        
g_InfoDataBase
    INCBIN ..\FwInfo\FwInfo_dumy.bin ; for allocate F/W info data region in NuBL2 F/W
g_InfoDataLimit

    END
