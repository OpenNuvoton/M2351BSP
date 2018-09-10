    AREA _NuBL32KeyStorage, DATA, READONLY

	EXPORT  g_NuBL32EnCryptPubKeyBase
    EXPORT  g_NuBL32EnCryptPubKeyBaseEnd
	EXPORT  g_NuBL33EnCryptPubKeyBase
    EXPORT  g_NuBL33EnCryptPubKeyBaseEnd
	EXPORT  g_NuBL32EnCryptPubKeyHashBase
    EXPORT  g_NuBL32EnCryptPubKeyHashBaseEnd
	EXPORT  g_NuBL33EnCryptPubKeyHashBase
    EXPORT  g_NuBL33EnCryptPubKeyHashBaseEnd
	EXPORT  g_HostEnCryptPubKeyBase
    EXPORT  g_HostEnCryptPubKeyBaseEnd
	EXPORT  g_HostEnCryptPubKeyHashBase
    EXPORT  g_HostEnCryptPubKeyHashBaseEnd

;Encrypted NuBL32 Public key
    ALIGN   4  
g_NuBL32EnCryptPubKeyBase
    INCBIN ..\keyInfo\NuBL32_key.bin
g_NuBL32EnCryptPubKeyBaseEnd

;Encrypted NuBL33 Public key
g_NuBL33EnCryptPubKeyBase
    INCBIN ..\keyInfo\NuBL33_key.bin
g_NuBL33EnCryptPubKeyBaseEnd

;Hash of Encrypted NuBL32 Public key
g_NuBL32EnCryptPubKeyHashBase
    INCBIN ..\keyInfo\NuBL32_keyHash.bin
g_NuBL32EnCryptPubKeyHashBaseEnd

;Hash of Encrypted NuBL33 Public key
g_NuBL33EnCryptPubKeyHashBase
    INCBIN ..\keyInfo\NuBL33_keyHash.bin
g_NuBL33EnCryptPubKeyHashBaseEnd

;Encrypted Host Public key
g_HostEnCryptPubKeyBase
    INCBIN ..\keyInfo\Host_key.bin
g_HostEnCryptPubKeyBaseEnd

;Hash of Encrypted Host Public key
g_HostEnCryptPubKeyHashBase
    INCBIN ..\keyInfo\Host_keyHash.bin
g_HostEnCryptPubKeyHashBaseEnd

    END