

    AREA _NuBL32KeyStorage, DATA, READONLY

    EXPORT  g_NuBL32KeyStart
    EXPORT  g_NuBL32KeyEnd
    EXPORT  g_NuBL32KeyHashStart
    EXPORT  g_NuBL32KeyHashEnd

    EXPORT  g_NuBL33KeyStart
    EXPORT  g_NuBL33KeyEnd
    EXPORT  g_NuBL33KeyHashStart
    EXPORT  g_NuBL33KeyHashEnd

    ALIGN   4
        
g_NuBL32KeyStart
    INCBIN ..\KeyInfo\NuBL32PubKeyEncrypted.bin
g_NuBL32KeyEnd

g_NuBL32KeyHashStart
    INCBIN ..\KeyInfo\NuBL32PubKeyEncryptedHash.bin
g_NuBL32KeyHashEnd


g_NuBL33KeyStart
    INCBIN ..\KeyInfo\NuBL33PubKeyEncrypted.bin
g_NuBL33KeyEnd

g_NuBL33KeyHashStart
    INCBIN ..\KeyInfo\NuBL33PubKeyEncryptedHash.bin
g_NuBL33KeyHashEnd

    END
